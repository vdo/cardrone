# Post-build: merge bootloader + partitions + boot_app0 + app into one image
# for web flashers (ESPConnect, esptool-js): flash fresneldrone.bin (fresneldrone-heltec.bin
# for the Heltec) at offset 0x0.
# Also stamps the firmware with `git describe` (v0.3 on a release tag,
# v0.3-2-gabc1234 between releases), shown on the boot screen.
import os
import subprocess
Import("env", "projenv")

try:
    version = subprocess.check_output(["git", "describe", "--tags", "--always", "--dirty"],
                                      cwd=env.subst("$PROJECT_DIR"), text=True).strip()
except Exception:
    version = "dev"
projenv.Append(CPPDEFINES=[("FRESNELDRONE_VERSION", env.StringifyMacro(version))])


def merge(source, target, env):
    name = "fresneldrone.bin" if env["PIOENV"] == "fresneldrone" else f"fresneldrone-{env['PIOENV']}.bin"
    out = os.path.join(env.subst("$PROJECT_DIR"), name)
    parts = [f"{off} \"{path}\"" for off, path in env["FLASH_EXTRA_IMAGES"]]  # bootloader, partitions, boot_app0
    parts.append(f"{env.subst('$ESP32_APP_OFFSET')} \"$BUILD_DIR/${{PROGNAME}}.bin\"")
    env.Execute(env.subst(
        f'"$PYTHONEXE" "$OBJCOPY" --chip esp32s3 merge_bin -o "{out}" '
        f'--flash_mode keep --flash_freq keep --flash_size keep {" ".join(parts)}'))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge)
