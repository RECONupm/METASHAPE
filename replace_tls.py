"""
Metashape TLS Replacement Helper (E57)

What this script does:
1) Shows a warning dialog (E57 axis conventions may differ between vendors/software).
2) Prompts for a folder containing .e57 files.
3) For each .e57 whose base filename matches an existing TLS station label in the active chunk:
   - Imports the .e57 as a laser scan.
   - Computes and applies a rigid transform DELTA so the imported scan matches the pose
     of the existing station (using effective transform = group.transform * pc.transform).
   - Removes any masks on the newly imported station cameras.
   - Copies masks from the ORIGINAL station cameras to the NEW station cameras.

Important:
- This assumes the input E57 point clouds were generated using the same software as the
  point clouds already in the Metashape project (otherwise yaw/axis conventions can differ).
"""

import os
import Metashape


# -----------------------------------------------------------------------------
# Small utility helpers
# -----------------------------------------------------------------------------

def _norm_name(s: str) -> str:
    """Normalize a name for robust matching (trim + lower)."""
    return (s or "").strip().lower()


def _get_e57_format():
    """Resolve the E57 point cloud format enum in a robust way across Metashape builds."""
    if hasattr(Metashape, "PointCloudFormatE57"):
        return Metashape.PointCloudFormatE57
    if hasattr(Metashape, "PointCloudFormat") and hasattr(Metashape.PointCloudFormat, "PointCloudFormatE57"):
        return Metashape.PointCloudFormat.PointCloudFormatE57
    raise RuntimeError("PointCloudFormatE57 was not found in this Metashape installation.")


def _make_unique_label(desired: str, existing_lower: set) -> str:
    """Ensure the label is unique inside the chunk by appending _02, _03, ..."""
    if desired.lower() not in existing_lower:
        return desired

    i = 2
    while True:
        candidate = f"{desired}_{i:02d}"
        if candidate.lower() not in existing_lower:
            return candidate
        i += 1


def _copy_matrix(M):
    """Best-effort deep copy for Metashape.Matrix-like objects."""
    if M is None:
        return None
    if hasattr(M, "copy"):
        return M.copy()
    try:
        return Metashape.Matrix(M)
    except Exception:
        return M


def _mat_to_str(M, digits: int = 6) -> str:
    """Pretty-print a 4x4 matrix for console debugging."""
    if M is None:
        return "None"
    rows = []
    for r in range(4):
        row = []
        for c in range(4):
            v = M[r, c]
            try:
                row.append(f"{float(v): .{digits}f}")
            except Exception:
                row.append(str(v))
        rows.append("[ " + "  ".join(row) + " ]")
    return "\n".join(rows)


def _effective_T(pc):
    """
    Effective transform used for pose comparisons when PointCloudGroup is involved:

        T_eff = group.transform * pc.transform

    If the point cloud has no group (or no group transform), then:

        T_eff = pc.transform
    """
    T = getattr(pc, "transform", None)
    if T is None:
        return None

    g = getattr(pc, "group", None)
    gT = getattr(g, "transform", None) if g is not None else None
    if gT is not None:
        return gT * T
    return T


def _apply_delta_to_pc_transform(pc, delta):
    """
    Apply the delta in a multiplicative way (do not overwrite the transform frame):

        pc.transform := delta * pc.transform
    """
    T = getattr(pc, "transform", None)
    if T is None:
        return
    pc.transform = delta * T


# -----------------------------------------------------------------------------
# Camera association + mask utilities
# -----------------------------------------------------------------------------

def _camera_belongs_to_pointcloud(cam, pc) -> bool:
    """
    Best-effort association: determine whether a camera belongs to a given point cloud (TLS).

    Metashape builds differ, so we try multiple possible link properties.
    """
    # Most direct (if exposed):
    try:
        if getattr(cam, "point_cloud", None) == pc:
            return True
    except Exception:
        pass

    # Some builds expose dense_cloud_id that may match point cloud key:
    try:
        if getattr(cam, "dense_cloud_id", None) == getattr(pc, "key", None):
            return True
    except Exception:
        pass

    return False


def _attached_cameras(chunk, pc):
    """
    Return cameras associated with the given TLS point cloud, sorted for stable pairing.
    """
    cams = []
    for cam in getattr(chunk, "cameras", []) or []:
        if _camera_belongs_to_pointcloud(cam, pc):
            cams.append(cam)

    cams.sort(key=lambda c: ((getattr(c, "label", "") or ""), getattr(c, "key", 0)))
    return cams


def _copy_mask(mask_obj):
    """
    Best-effort deep copy for a Metashape mask.
    """
    if mask_obj is None:
        return None
    if hasattr(mask_obj, "copy"):
        try:
            return mask_obj.copy()
        except Exception:
            pass
    # Fallback: try constructor copy
    try:
        return Metashape.Mask(mask_obj)
    except Exception:
        return mask_obj


def _clear_camera_mask(cam) -> bool:
    """
    Clear/remove the mask from a camera in a robust way.
    Returns True if we successfully cleared something.
    """
    if hasattr(cam, "mask"):
        try:
            cam.mask = None
            return True
        except Exception:
            pass

        # Some builds may keep a mask object that supports clear()
        try:
            m = getattr(cam, "mask", None)
            if m is not None and hasattr(m, "clear"):
                m.clear()
                cam.mask = m
                return True
        except Exception:
            pass

    # Rare fallback
    if hasattr(cam, "resetMask"):
        try:
            cam.resetMask()
            return True
        except Exception:
            pass

    return False


def _transfer_masks_from_src_to_new(chunk, src_pc, new_pc):
    """
    Remove masks on NEW station cameras, then copy masks from SRC station cameras.

    Pairing strategy:
    - Collect cameras attached to src_pc and new_pc (best-effort association).
    - Sort cameras by (label, key) for stable deterministic pairing.
    - Copy mask by index: src_cams[i].mask -> new_cams[i].mask

    If counts differ, copy the common subset and print a warning.
    """
    src_cams = _attached_cameras(chunk, src_pc)
    new_cams = _attached_cameras(chunk, new_pc)

    print(f"  Cameras attached | SRC: {len(src_cams)} | NEW: {len(new_cams)}")

    # Always clear masks on NEW cameras first (as requested previously)
    cleared = 0
    for cam in new_cams:
        if _clear_camera_mask(cam):
            cleared += 1
    if new_cams:
        print(f"  Cleared masks on NEW cameras: {cleared}/{len(new_cams)}")

    if not src_cams or not new_cams:
        print("  WARNING: Cannot transfer masks (missing SRC cameras or NEW cameras).")
        return

    n = min(len(src_cams), len(new_cams))
    copied = 0
    for i in range(n):
        src_mask = getattr(src_cams[i], "mask", None)
        if src_mask is None:
            # Nothing to copy for this camera
            continue
        try:
            new_cams[i].mask = _copy_mask(src_mask)
            copied += 1
        except Exception:
            # If assignment fails, keep going
            pass

    if len(src_cams) != len(new_cams):
        print(f"  WARNING: Camera count mismatch (SRC={len(src_cams)} NEW={len(new_cams)}). "
              f"Transferred masks for first {n} camera pairs.")
    print(f"  Masks copied to NEW cameras: {copied}/{n}")


# -----------------------------------------------------------------------------
# Main workflow
# -----------------------------------------------------------------------------

def importar_e57_y_aplicar_delta_y_mascaras():
    """
    Full pipeline:
    - Import matching E57 as laser scans
    - Apply transform DELTA to match existing station pose
    - Clear masks on imported stations
    - Copy masks from original stations to imported stations
    - Print matrices for debugging
    """
    doc = Metashape.app.document
    if not doc or not doc.chunk:
        raise RuntimeError("No active Metashape document/chunk found.")

    chunk = doc.chunk

    # Warning dialog BEFORE asking for the folder
    Metashape.app.messageBox(
        "Warning:\n\n"
        "The input point clouds must be generated using the same software as the "
        "point clouds already loaded in this Metashape project.\n\n"
        "Otherwise, different axis conventions (e.g., yaw) may cause incorrect orientations."
    )

    folder = Metashape.app.getExistingDirectory("Select the folder containing .e57 files")
    if not folder:
        print("Operation cancelled (no folder selected).")
        return

    e57_files = sorted([
        f for f in os.listdir(folder)
        if os.path.isfile(os.path.join(folder, f)) and f.lower().endswith(".e57")
    ])
    if not e57_files:
        print(f"No .e57 files found in: {folder}")
        return

    pcs = getattr(chunk, "point_clouds", None) or []
    laser_scans = [pc for pc in pcs if getattr(pc, "is_laser_scan", False)]
    if not laser_scans:
        print("No TLS laser scans found in the active chunk.")
        return

    # Map existing stations by normalized label
    loaded_by_name = {}
    for pc in laser_scans:
        k = _norm_name(getattr(pc, "label", ""))
        if not k:
            continue
        if k in loaded_by_name:
            print(f"WARNING: Duplicate scan label '{pc.label}'. The first one will be used.")
            continue
        loaded_by_name[k] = pc

    existing_labels_lower = {_norm_name(getattr(pc, "label", "")) for pc in pcs if getattr(pc, "label", "")}
    fmt_e57 = _get_e57_format()

    print(f"Chunk: {chunk.label!r}")
    print(f"Folder: {folder}")
    print(f"E57 files found: {len(e57_files)}")
    print(f"TLS scans in chunk: {len(laser_scans)}\n")

    for fname in e57_files:
        base = os.path.splitext(fname)[0]
        key = _norm_name(base)

        if key not in loaded_by_name:
            continue

        src = loaded_by_name[key]
        if getattr(src, "transform", None) is None:
            print(f"SKIP '{fname}': '{src.label}' has no transform.")
            continue

        print("\n" + "=" * 90)
        print(f"MATCH: '{fname}' <-> '{src.label}' (src key={src.key})")

        # Source transforms
        T_src_eff = _copy_matrix(_effective_T(src))
        T_src_pc = _copy_matrix(getattr(src, "transform", None))

        if T_src_eff is None:
            print("SKIP: Could not compute source effective transform (T_src_eff).")
            continue

        print("\n(1) ORIGINAL POINT CLOUD IN METASHAPE")
        print("SRC pc.transform:")
        print(_mat_to_str(T_src_pc))
        print("SRC effective (group*pc if applicable):")
        print(_mat_to_str(T_src_eff))

        # Import
        before_keys = {pc.key for pc in (getattr(chunk, "point_clouds", None) or [])}
        chunk.importPointCloud(
            path=os.path.join(folder, fname),
            format=fmt_e57,
            is_laser_scan=True,
            replace_asset=False
        )

        after_pcs = getattr(chunk, "point_clouds", None) or []
        new_pcs = [pc for pc in after_pcs if pc.key not in before_keys]
        if not new_pcs:
            print("ERROR: No new PointCloud assets detected after import.")
            continue

        # Process each imported PC (some E57 may generate multiple assets)
        for idx, new_pc in enumerate(new_pcs, start=1):

            # Labeling
            desired = f"{base}_new" if idx == 1 else f"{base}_new_{idx:02d}"
            new_label = _make_unique_label(desired, existing_labels_lower)
            new_pc.label = new_label
            existing_labels_lower.add(_norm_name(new_pc.label))

            # Put imported scan in the same group as the source (for consistent effective transforms)
            try:
                if getattr(src, "group", None) is not None:
                    new_pc.group = src.group
            except Exception:
                pass

            # (2) Imported transforms (raw)
            T_new0_pc = _copy_matrix(getattr(new_pc, "transform", None))
            T_new0_eff = _copy_matrix(_effective_T(new_pc))

            print("\n(2) IMPORTED POINT CLOUD (RAW)")
            print(f"IMPORTED pc.transform (label='{new_pc.label}', key={new_pc.key}):")
            print(_mat_to_str(T_new0_pc))
            print("IMPORTED effective (group*pc if applicable):")
            print(_mat_to_str(T_new0_eff))

            if T_new0_eff is None:
                print("ERROR: Could not compute imported effective transform (T_new0_eff).")
                continue

            # Delta so that DELTA * imported_eff == src_eff
            delta = T_src_eff * T_new0_eff.inv()

            print("\nDELTA = SRC_eff * inv(IMPORTED_eff):")
            print(_mat_to_str(delta))

            # Apply delta
            _apply_delta_to_pc_transform(new_pc, delta)

            # (3) Final transforms
            T_newF_pc = _copy_matrix(getattr(new_pc, "transform", None))
            T_newF_eff = _copy_matrix(_effective_T(new_pc))

            print("\n(3) IMPORTED POINT CLOUD (FINAL AFTER DELTA)")
            print("FINAL pc.transform:")
            print(_mat_to_str(T_newF_pc))
            print("FINAL effective (group*pc if applicable):")
            print(_mat_to_str(T_newF_eff))

            # Operational: keep enabled state
            try:
                new_pc.enabled = src.enabled
            except Exception:
                pass

            # --- Mask transfer: clear NEW masks, then copy masks from SRC station to NEW station ---
            print("\n(4) MASK TRANSFER (SRC -> NEW)")
            _transfer_masks_from_src_to_new(chunk, src, new_pc)

        print("=" * 90)


# Run
importar_e57_y_aplicar_delta_y_mascaras()
