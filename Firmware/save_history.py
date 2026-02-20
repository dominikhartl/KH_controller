"""
PlatformIO extra_scripts: preserve history files across LittleFS uploads.

Before uploadfs: downloads history CSVs from the device into data/history/
so they are included in the new LittleFS image.
After uploadfs: cleans up data/history/ to avoid committing stale data.
"""
import os
import shutil
Import("env")

HISTORY_FILES = ["kh", "ph", "gran"]
DATA_DIR = os.path.join(env.subst("$PROJECT_DIR"), "data", "history")
DEVICE_HOST = "khcontrollerv3.local"


def backup_history(source, target, env):
    """Download history files from device before building filesystem image."""
    import urllib.request

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


def cleanup_history(source, target, env):
    """Remove data/history/ after upload to avoid stale files in repo."""
    if os.path.exists(DATA_DIR):
        shutil.rmtree(DATA_DIR)
        print("  Cleaned up data/history/")


# Hook into the uploadfs target
env.AddPreAction("uploadfs", backup_history)
env.AddPostAction("uploadfs", cleanup_history)
