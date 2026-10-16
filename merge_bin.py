# Post-build: merge bootloader + partitions + boot_app0 + app into one image
# for web flashers (ESPConnect, esptool-js): flash cardrone.bin at offset 0x0.
import os
Import("env")


def merge(source, target, env):
    out = os.path.join(env.subst("$PROJECT_DIR"), "cardrone.bin")
    parts = [f"{off} \"{path}\"" for off, path in env["FLASH_EXTRA_IMAGES"]]  # bootloader, partitions, boot_app0
    parts.append(f"{env.subst('$ESP32_APP_OFFSET')} \"$BUILD_DIR/${{PROGNAME}}.bin\"")
    env.Execute(env.subst(
        f'"$PYTHONEXE" "$OBJCOPY" --chip esp32s3 merge_bin -o "{out}" '
        f'--flash_mode keep --flash_freq keep --flash_size keep {" ".join(parts)}'))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", merge)
