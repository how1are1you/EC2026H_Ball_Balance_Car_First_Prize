# Repository Guidelines

## Project Structure & Module Organization

This is a bare-metal C firmware project for the TI MSPM0G3507 (Cortex-M0+).
`empty.c` is the application entry point, and `empty.syscfg` defines the TI
SysConfig peripheral setup that generates `ti_msp_dl_config.c/.h`. Put vehicle
control and protocol logic in `Control/`; put board-facing drivers (motor,
encoder, OLED, ADC, LEDs, keys, and CCD) in `Hardware/`. The `ti/` directory
contains DriverLib sources required by this project. `source/` is the bundled
TI SDK and third-party material; avoid modifying it unless updating a vendored
dependency. Keil project settings, linker script, startup assembly, and build
output live under `keil/`.

## Build, Test, and Development Commands

Open `keil/empty_LP_MSPM0G3507_nortos_keil.uvprojx` in Keil uVision, select the
`MSPM0G3507_Project` target, then use **Project > Build Target**. This produces
the firmware image under `keil/Objects/` (for example, the `.axf` file). If
uVision is on `PATH`, the equivalent command is:

```powershell
UV4.exe -b keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx -t MSPM0G3507_Project
```

After changing `empty.syscfg`, set `SYSCFG_PATH` in `tools/keil/syscfg.bat` to
the installed SysConfig CLI and regenerate `ti_msp_dl_config.c/.h` before
building. Do not edit generated configuration files by hand.

## Coding Style & Naming Conventions

Use C99 and follow the nearby module's style: four-space indentation in new
code, braces on their own line for functions and control blocks, and a matching
`.c`/`.h` pair for each driver. Use descriptive `snake_case` function names;
retain existing hardware and TI names such as `TIMER_0_INST_IRQHandler` and
`DL_Timer_setCaptureCompareValue`. Keep interrupt handlers short, avoid dynamic
allocation, and mark shared ISR/main-loop state `volatile` when appropriate.

## Testing Guidelines

There is no automated test framework or coverage target. A change is not ready
until the Keil build completes without new warnings and it has been checked on
the target board. Test affected peripherals and record the board, firmware
mode, and observed serial/OLED/motor behavior in the PR description.

## Commit & Pull Request Guidelines

The history currently contains only the initial baseline commit, so no commit
message convention is established. Use concise imperative subjects, preferably
scoped, such as `motor: clamp reverse PWM`. Keep commits focused. PRs should
describe the behavior change, list hardware verification, link relevant issues,
and include serial output or photos for visible hardware behavior. Do not add
new generated files from `keil/Objects/`, local uVision user settings, or
private keys.
