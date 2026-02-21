"""
PlatformIO extra_scripts: preserve history files across LittleFS uploads.

Downloads history CSVs from the device into data/history/ BEFORE the LittleFS
image is built, so they are included. Cleans up after upload.

Key: the download runs at script load time (during SCons configuration),
not as a pre-action callback. Pre-actions on "uploadfs" run AFTER the
littlefs.bin dependency is already built — too late.
"""
import os
import shutil
from SCons.Script import COMMAND_LINE_TARGETS

Import("env")

HISTORY_FILES = ["kh", "ph", "gran"]
DATA_DIR = os.path.join(env.subst("$PROJECT_DIR"), "data", "history")
DEVICE_HOST = "khcontrollerv3.local"


def cleanup_history(source, target, env):
    """Remove data/history/ after upload to avoid stale files in repo."""
    if os.path.exists(DATA_DIR):
        shutil.rmtree(DATA_DIR)
        print("  Cleaned up data/history/")


# Download history at script load time — runs BEFORE any targets are built
if "uploadfs" in COMMAND_LINE_TARGETS:
    import urllib.request

    print("Backing up history from device before filesystem build...")
    os.makedirs(DATA_DIR, exist_ok=True)
    for name in HISTORY_FILES:
        url = f"http://{DEVICE_HOST}/api/history/{name}"
        dest = os.path.join(DATA_DIR, f"{name}.csv")
        try:
            req = urllib.request.urlopen(url, timeout=5)
            if req.status == 200:
                data = req.read()
                if len(data) > 0:
                    with open(dest, "wb") as f:
                        f.write(data)
                    print(f"  Backed up {name}.csv ({len(data)} bytes)")
                else:
                    print(f"  {name}.csv is empty, skipping")
            else:
                print(f"  {name}.csv not found (HTTP {req.status})")
        except Exception as e:
            print(f"  Could not backup {name}.csv: {e}")

    # Delete cached LittleFS image to force rebuild with history files
    img = os.path.join(env.subst("$BUILD_DIR"), "littlefs.bin")
    if os.path.exists(img):
        os.remove(img)
        print("  Removed cached littlefs.bin to force rebuild with history")

env.AddPostAction("uploadfs", cleanup_history)
