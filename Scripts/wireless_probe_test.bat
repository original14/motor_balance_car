@echo off
setlocal EnableExtensions

rem CMSIS-DAP/SWD connection test for MSPM0G3507. This script never writes Flash.
rem Optional per-invocation overrides:
rem   set "OPENOCD_EXE=C:\path\to\openocd.exe"
rem   set "OPENOCD_SCRIPTS=C:\path\to\openocd\scripts"
rem   set "SWD_SPEED_KHZ=500"

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "PROJECT_DIR=%%~fI"
if not defined OPENOCD_EXE set "OPENOCD_EXE=D:\ti\openocd\bin\openocd.exe"
if not defined OPENOCD_SCRIPTS set "OPENOCD_SCRIPTS=D:\ti\openocd\share\openocd\scripts"
if not defined SWD_SPEED_KHZ set "SWD_SPEED_KHZ=100"

if not exist "%OPENOCD_EXE%" (
    echo [ERROR] OPENOCD_EXE does not exist: "%OPENOCD_EXE%"
    exit /b 2
)

echo [INFO] Project: "%PROJECT_DIR%"
echo [INFO] OpenOCD: "%OPENOCD_EXE%"
"%OPENOCD_EXE%" --version
if errorlevel 1 (
    echo [ERROR] OpenOCD could not report its version.
    exit /b 6
)
echo [INFO] OpenOCD scripts: "%OPENOCD_SCRIPTS%"
if not exist "%OPENOCD_SCRIPTS%\interface\cmsis-dap.cfg" (
    echo [ERROR] Missing interface\cmsis-dap.cfg under "%OPENOCD_SCRIPTS%"
    exit /b 3
)
if not exist "%OPENOCD_SCRIPTS%\target\ti_mspm0.cfg" (
    echo [ERROR] Missing target\ti_mspm0.cfg under "%OPENOCD_SCRIPTS%"
    echo [ERROR] This OpenOCD installation is not ready for MSPM0G3507.
    exit /b 4
)

echo [INFO] Adapter: CMSIS-DAP over SWD
echo [INFO] SWD speed: %SWD_SPEED_KHZ% kHz
echo [WARNING] This test does not write Flash, but it resets and halts the MCU briefly.
echo [COMMAND] "%OPENOCD_EXE%" -s "%OPENOCD_SCRIPTS%" -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed %SWD_SPEED_KHZ%" -f target/ti_mspm0.cfg -c "init; reset halt; shutdown"
echo [INFO] Starting Wireless Probe Test...

"%OPENOCD_EXE%" ^
  -s "%OPENOCD_SCRIPTS%" ^
  -f interface/cmsis-dap.cfg ^
  -c "transport select swd" ^
  -c "adapter speed %SWD_SPEED_KHZ%" ^
  -f target/ti_mspm0.cfg ^
  -c "init; reset halt; shutdown"

set "OPENOCD_RC=%ERRORLEVEL%"
if not "%OPENOCD_RC%"=="0" (
    echo [ERROR] Wireless Probe Test failed. OpenOCD exit code: %OPENOCD_RC%
    echo [HINT] Check Host connection, SWDIO, SWCLK, GND, NRST, target power, and Host/Slave pairing.
    exit /b %OPENOCD_RC%
)

echo [OK] Wireless Probe Test completed successfully. No Flash was written.
exit /b 0
