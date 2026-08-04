# GCC Makefile Build Guide

This project is still primarily maintained with Keil5, HAL, and CubeMX. The root `Makefile` adds a command-line GCC build path without changing business source logic.

## Toolchain

The Makefile uses the MSYS2 GCC toolchain. On this Windows make path it is invoked through the Windows-equivalent path of `/d/Msys2/mingw64/bin/arm-none-eabi-gcc`:

```sh
D:/Msys2/mingw64/bin/arm-none-eabi-gcc.exe
```

Related tools are taken from the same folder:

```sh
D:/Msys2/mingw64/bin/arm-none-eabi-objcopy.exe
D:/Msys2/mingw64/bin/arm-none-eabi-size.exe
```

## Files Added For GCC

- `Makefile`: root command-line build entry.
- `GCC/STM32H723VGTx_FLASH.ld`: GNU ld linker script matching the Keil memory layout.
- `GCC/startup_stm32h723xx_gcc.s`: GCC syntax startup file and interrupt vector table.
- `GCC_Makefile_Build_Guide.md`: this guide.
- `DSP/Include`, `DSP/PrivateInclude`, `DSP/Source`: project-local CMSIS-DSP source package used by both Keil and the GCC Makefile build.

The Keil startup file `MDK-ARM/startup_stm32h723xx.s` is ARMASM syntax and cannot be assembled by GCC directly, so the GCC build path uses its own startup file.

## CMSIS-DSP Integration

CMSIS-DSP is imported as project-local source code, not as a prebuilt `.lib` file. The source package is stored under:

```text
DSP/Include
DSP/PrivateInclude
DSP/Source
```

The Makefile adds these include paths:

```make
-IDSP/Include
-IDSP/PrivateInclude
```

It also compiles the non-F16 aggregate source files from `DSP/Source`, such as:

```text
DSP/Source/FastMathFunctions/FastMathFunctions.c
DSP/Source/CommonTables/CommonTables.c
DSP/Source/TransformFunctions/TransformFunctions.c
```

Do not add both an aggregate source file and the individual `arm_xxx.c` files from the same DSP module to the build at the same time, because that can create duplicate symbol definitions.

`arm_math.h` provides the public CMSIS-DSP declarations and compile-time configuration hooks. Functions such as `arm_sin_f32()` are linked from the compiled DSP source objects; for `arm_sin_f32()`, the relevant implementation path is `FastMathFunctions.c`, and lookup tables are provided by `CommonTables.c`.

The Makefile defines `DISABLEFLOAT16` for the GCC path. This project targets Cortex-M7 float32 usage, and the current Makefile intentionally excludes the F16 aggregate source files to keep the command-line build portable.

When sharing the project, include the whole `DSP/` directory. The GCC build no longer depends on another machine having CMSIS-DSP installed under a Keil Pack path such as `D:\keil_v543a\MDK5\ARM\CMSIS-DSP\...`.

## Build Output Archive Layout

All GCC build outputs are under `build/`:

```text
build/
  CtrBoard-H7-AIBOT-DPD.elf
  CtrBoard-H7-AIBOT-DPD.map
  CtrBoard-H7-AIBOT-DPD.hex
  CtrBoard-H7-AIBOT-DPD.bin
  obj/
    ... mirrored source tree containing .o and .d files
```

`build/` is ignored by Git through `.gitignore`.

## Build Commands

Open PowerShell or another command prompt where `make` is available. This machine currently resolves `make` from `D:\MinGW-posix\mingw64-posix\bin\make.exe`. Then enter the project root:

```powershell
cd D:\Internship\TsingHuaSZ-AI-Robot-LAB\4WIS4WID\TsingHuaSZ-AIRobot-SteeringWheel
```

Compile the project:

```sh
make
```

The default Makefile output is quiet and prints compact build steps such as `[CC]`, `[LD]`, `[HEX]`, and `[BIN]`. Use verbose mode when the full compiler and linker command lines are needed:

```sh
make V=1
```

The ELF output is:

```text
build/CtrBoard-H7-AIBOT-DPD.elf
```

Clean all GCC build outputs:

```sh
make clean
```

## Keil5 Build vs GCC Makefile Build

Keil5 build:

- Uses `MDK-ARM/CtrBoard-H7-AIBOT-DPD.uvprojx`.
- Uses ARM Compiler 6 / ARMCLANG settings stored in the Keil project.
- Uses Keil ARMASM startup and Keil scatter file/output conventions.
- Remains the main development and debugging environment.

GCC Makefile build:

- Uses root `Makefile` from the command line.
- Uses `D:/Msys2/mingw64/bin/arm-none-eabi-gcc.exe`.
- Uses GNU assembler syntax startup and GNU ld linker script under `GCC/`.
- Places every generated `.o`, `.d`, `.elf`, `.map`, `.hex`, and `.bin` file under `build/`.
- Is useful for command-line compilation, CI-style checks, and independent build verification.

## Source Logic Policy

This GCC build path does not modify application, BSP, middleware, HAL, interrupt, Modbus, motor-control, or safety-stop business logic.



