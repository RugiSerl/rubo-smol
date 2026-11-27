#!/usr/bin/env python3
"""
Build the robot firmware.

    python3 tools/build.py            build
    python3 tools/build.py --clean    build from scratch
    python3 tools/build.py --flash    build, then put it on the robot

The result is build/robot.uf2. --flash uses tools/pico.py to reboot a running
Pico into BOOTSEL mode and load it. A crashed program may need manual BOOTSEL.

Before building, the JavaScript in robot/web/ is checked for syntax errors
with Node. A module that does not parse stops the whole phone page from
loading, while the robot still boots and shows its address.

The build also updates VS Code's C/C++ configuration from CMake's
compile_commands.json.
"""

import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
SDK = REPO / ".pico-sdk"
SKETCH = REPO / "robot"
BUILD = REPO / "build"
WEB = SKETCH / "web"
UF2 = BUILD / "robot.uf2"


def require_sdk():
    if not (SDK / "pico_sdk_init.cmake").exists():
        sys.exit("Pico SDK missing -- run: python3 tools/setup.py")


def syntax_error(source, name):
    """Node's report on source as an ES module, or None if it parses."""
    # Through stdin with --input-type=module: Node 20 runs `node --check` on a
    # .js file as CommonJS, and passes a broken ES module without a word.
    result = subprocess.run(["node", "--input-type=module", "--check"],
                            input=source, capture_output=True, text=True)
    if result.returncode == 0:
        return None
    # Keep the location, the line, the caret and the message; drop the stack
    # trace, which points inside Node.
    report = []
    for line in result.stderr.replace("[stdin]", name).splitlines():
        if line.startswith("    at "):
            break
        report.append(line)
    return "\n".join(report).strip()


def check_web():
    """Refuse to build a phone page whose JavaScript does not parse."""
    if shutil.which("node") is None:
        print("note: node not found, robot/web/ JavaScript not checked")
        return

    errors = []
    for path in sorted(WEB.glob("*.js")):
        error = syntax_error(path.read_text(), str(path.relative_to(REPO)))
        if error:
            errors.append(error)

    # The <script type="module"> blocks inside the HTML pages. Each is padded
    # with the newlines that come before it, so line numbers match the file.
    for path in sorted(WEB.glob("*.html")):
        text = path.read_text()
        for match in re.finditer(r'<script type="module">(.*?)</script>', text, re.DOTALL):
            padding = "\n" * text.count("\n", 0, match.start(1))
            error = syntax_error(padding + match.group(1), str(path.relative_to(REPO)))
            if error:
                errors.append(error)

    if errors:
        print("\n\n".join(errors))
        print()
        sys.exit("JavaScript error in robot/web/: the phone page would not load. "
                 "Fix the line above and build again.")


def configure():
    """Ask CMake to work out the build. Only needed once, or after --clean."""
    subprocess.run(
        ["cmake",
         "-S", str(SKETCH),
         "-B", str(BUILD),
         "-G", "Ninja",
         "-DCMAKE_BUILD_TYPE=Release",
         "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON"],
        cwd=REPO, check=True,
    )


def build():
    subprocess.run(["ninja", "-C", str(BUILD)], cwd=REPO, check=True)


def report_size():
    """Print how much of the chip we are using, in terms that mean something."""
    elf = BUILD / "robot.elf"
    if not elf.exists() or shutil.which("arm-none-eabi-size") is None:
        return

    output = subprocess.run(["arm-none-eabi-size", str(elf)],
                            capture_output=True, text=True, check=True).stdout
    numbers = output.strip().splitlines()[-1].split()
    text, data, bss = int(numbers[0]), int(numbers[1]), int(numbers[2])

    flash = text + data           # the program itself, kept in flash
    ram = data + bss              # global variables, kept in RAM

    FLASH_TOTAL = 2 * 1024 * 1024
    RAM_TOTAL = 264 * 1024

    print()
    print(f"  program  {flash:>7,} bytes of {FLASH_TOTAL:,}  ({100*flash/FLASH_TOTAL:.0f}% of flash)")
    print(f"  globals  {ram:>7,} bytes of {RAM_TOTAL:,}  ({100*ram/RAM_TOTAL:.0f}% of RAM)")
    print(f"  -> {UF2.relative_to(REPO)}")


def intellisense():
    """Point the VS Code C/C++ extension at CMake's compile_commands.json."""
    database = BUILD / "compile_commands.json"
    if not database.exists():
        return

    config = {
        "version": 4,
        "configurations": [{
            "name": "Pico (Pico SDK)",
            "compileCommands": str(database),
            "compilerPath": shutil.which("arm-none-eabi-gcc") or "",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-arm",
        }],
    }

    target = REPO / ".vscode" / "c_cpp_properties.json"
    target.parent.mkdir(exist_ok=True)
    header = "// Generated by tools/build.py. Do not edit; re-run the script instead.\n"
    target.write_text(header + json.dumps(config, indent=2) + "\n")


def flash():
    """Hand over to tools/pico.py, which is where everything USB lives."""
    print()
    # pico.py writes straight to the terminal, so anything we have printed has
    # to be out of the buffer first or the two get interleaved out of order.
    sys.stdout.flush()
    # Not check=True: pico.py has already explained itself in plain words, and
    # a Python traceback on top of that helps nobody.
    result = subprocess.run([sys.executable, str(REPO / "tools" / "pico.py"), "flash"])
    if result.returncode != 0:
        sys.exit(result.returncode)


def main():
    require_sdk()
    check_web()

    if "--clean" in sys.argv and BUILD.exists():
        shutil.rmtree(BUILD)

    # CMake only needs to run again when the build folder is missing or the
    # CMakeLists.txt changed; ninja re-runs it automatically in that case.
    if not (BUILD / "build.ninja").exists():
        configure()

    build()
    intellisense()
    report_size()

    if "--flash" in sys.argv:
        flash()


if __name__ == "__main__":
    main()
