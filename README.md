# METASHAPE

Metashape utility script for replacing TLS station assets with new E57 imports while
preserving station pose alignment and camera masks.

## What the script does

The `replace_tls.py` script:

1. Warns about potential axis-convention mismatches between vendors.
2. Prompts for a folder containing `.e57` files.
3. Finds matching TLS stations in the active Metashape chunk by filename.
4. Imports matching E57 files as laser scans.
5. Computes and applies a rigid transform so the imported scan matches the original
   station pose (using the effective transform of point cloud groups).
6. Clears masks on the newly imported station cameras.
7. Copies masks from the original station cameras to the new station cameras.

## Requirements

- Agisoft Metashape installed with Python scripting enabled.
- A Metashape project open with an active chunk containing TLS laser scans.
- E57 files named to match existing TLS station labels (case-insensitive).

## Usage

1. Open your Metashape project.
2. Ensure the active chunk contains TLS stations with labels that match the E57 base
   filenames.
3. Run the script from Metashape:
   - **Tools → Run Script…** and select `replace_tls.py`, or
   - Execute it from the Metashape Python console.
4. Select the folder containing the E57 files when prompted.

The script prints debug information to the console, including the original and final
transforms and mask transfer results.

## Notes

- The script skips E57 files whose base filename does not match an existing TLS station
  label.
- If multiple assets are imported from a single E57 file, the script appends suffixes
  to keep point cloud labels unique.
