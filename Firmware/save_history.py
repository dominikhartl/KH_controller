"""
PlatformIO extra_scripts: preserve history files across LittleFS uploads
and pre-compress web assets for faster page loads.

Downloads history CSVs from the device into data/history/ BEFORE the LittleFS
image is built, so they are included. Also gzips web assets (HTML/JS/CSS) and
removes originals so ESPAsyncWebServer serves compressed versions (it only
serves .gz when the original file is absent). Restores originals after upload.

Key: the download runs at script load time (during SCons configuration),
not as a pre-action callback. Pre-actions on "uploadfs" run AFTER the
littlefs.bin dependency is already built — too late.
"""
import os
import gzip
import shutil
import subprocess
from SCons.Script import COMMAND_LINE_TARGETS

Import("env")

HISTORY_FILES = ["kh", "ph", "gran", "precision"]
DATA_DIR = os.path.join(env.subst("$PROJECT_DIR"), "data", "history")
WWW_DIR = os.path.join(env.subst("$PROJECT_DIR"), "data", "www")
DEVICE_HOST = env.subst("$UPLOAD_PORT") or "khpro.local"

# Web assets to gzip (source files kept in git, .gz generated at build time)
GZIP_EXTENSIONS = (".html", ".js", ".css")

# Track which originals were backed up so we can restore them
_backed_up = {}  # fname -> backup_path


def gzip_web_assets():
    """Compress web assets and remove originals for LittleFS build."""
    for fname in os.listdir(WWW_DIR):
        if any(fname.endswith(ext) for ext in GZIP_EXTENSIONS):
            src = os.path.join(WWW_DIR, fname)
            dst = src + ".gz"
            # Regenerate if source is newer than .gz or .gz doesn't exist
            if not os.path.exists(dst) or os.path.getmtime(src) > os.path.getmtime(dst):
                with open(src, "rb") as f_in:
                    with gzip.open(dst, "wb", compresslevel=9) as f_out:
                        f_out.write(f_in.read())
                orig = os.path.getsize(src)
                comp = os.path.getsize(dst)
                print(f"  Gzipped {fname}: {orig} -> {comp} bytes ({100-comp*100//orig}% smaller)")

    # Back up and remove originals — ESPAsyncWebServer only serves .gz when original is absent
    for fname in list(os.listdir(WWW_DIR)):
        if any(fname.endswith(ext) for ext in GZIP_EXTENSIONS):
            src = os.path.join(WWW_DIR, fname)
            gz_path = src + ".gz"
            if os.path.exists(gz_path):
                bak = src + ".bak"
                shutil.copy2(src, bak)
                os.remove(src)
                _backed_up[fname] = bak
    if _backed_up:
        print(f"  Removed originals for build: {', '.join(_backed_up.keys())}")


def restore_originals():
    """Restore original web assets from backups after upload."""
    for fname, bak in _backed_up.items():
        dst = os.path.join(WWW_DIR, fname)
        if os.path.exists(bak):
            shutil.move(bak, dst)
    if _backed_up:
        print(f"  Restored originals: {', '.join(_backed_up.keys())}")


def cleanup_after_upload(source, target, env):
    """Remove data/history/ and restore web assets after upload."""
    if os.path.exists(DATA_DIR):
        shutil.rmtree(DATA_DIR)
        print("  Cleaned up data/history/")
    restore_originals()


# Gzip web assets at script load time (before LittleFS image is built)
if "uploadfs" in COMMAND_LINE_TARGETS:
    print("Compressing web assets...")
    gzip_web_assets()

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

env.AddPostAction("uploadfs", cleanup_after_upload)
