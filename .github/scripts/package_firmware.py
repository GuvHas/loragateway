"""Collect the flashable images for one PlatformIO environment into dist/.

Writes:
  dist/firmware.bin       application image only, for updating a board that
                          already has this project's bootloader and partition
                          table (flash at the application offset, or use OTA)
  dist/firmware-full.bin  bootloader, partition table, boot_app0 and the
                          application merged into one image, for a blank or
                          erased board (flash at 0x0)
  dist/flash-offsets.txt  the offset of every image in firmware-full.bin

The offsets and image paths come from PlatformIO's own project metadata, so
they follow the platform version and partition table instead of being
hard-coded here.

Usage: python package_firmware.py <environment>
"""

import json
import shutil
import subprocess
import sys
from pathlib import Path


def main() -> int:
    env = sys.argv[1]
    metadata = json.loads(
        subprocess.check_output(
            ["pio", "project", "metadata", "-e", env, "--json-output"], text=True
        )
    )[env]
    extra = metadata.get("extra", {})
    images = extra.get("flash_images") or []
    app_offset = extra.get("application_offset")
    app = Path(metadata["prog_path"]).with_suffix(".bin")
    if not images or not app_offset or not app.is_file():
        print(f"Missing flash layout in PlatformIO metadata: {extra}", file=sys.stderr)
        return 1

    layout = [(img["offset"], Path(img["path"])) for img in images]
    layout.append((app_offset, app))
    layout.sort(key=lambda item: int(item[0], 16))

    dist = Path("dist")
    dist.mkdir(exist_ok=True)
    shutil.copy(app, dist / "firmware.bin")

    merge = ["esptool.py", "--chip", "esp32", "merge_bin", "-o", str(dist / "firmware-full.bin")]
    for offset, path in layout:
        merge += [offset, str(path)]
    subprocess.check_call(merge)

    lines = [f"{offset}  {path.name}" for offset, path in layout]
    (dist / "flash-offsets.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    return 0


if __name__ == "__main__":
    sys.exit(main())
