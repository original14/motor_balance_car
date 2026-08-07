@echo off
setlocal EnableExtensions

rem CMSIS-DAP/SWD Flash programming for MSPM0G3507.
rem Optional per-invocation overrides:
rem   set "OPENOCD_EXE=C:\path\to\openocd.exe"
rem   set "OPENOCD_SCRIPTS=C:\path\to\openocd\scripts"
rem   set "SWD_SPEED_KHZ=100"
rem   set "FIRMWARE_OVERRIDE=C:\path\to\firmware.out"

set "SCRIPT_DIR=%~dp0"
for %%I in ("%SCRIPT_DIR%..") do set "PROJECT_DIR=%%~fI"
if not defined OPENOCD_EXE set "OPENOCD_EXE=D:\ti\openocd\bin\openocd.exe"
if not defined OPENOCD_SCRIPTS set "OPENOCD_SCRIPTS=D:\ti\openocd\share\openocd\scripts"
if not defined SWD_SPEED_KHZ set "SWD_SPEED_KHZ=500"

set "FIRMWARE="
if defined FIRMWARE_OVERRIDE set "FIRMWARE=%FIRMWARE_OVERRIDE%"
if defined FIRMWARE goto firmware_found
call :find_firmware "%PROJECT_DIR%\Debug" "*.out"
if defined FIRMWARE goto firmware_found
call :find_firmware "%PROJECT_DIR%\Debug" "*.elf"
if defined FIRMWARE goto firmware_found
call :find_firmware "%PROJECT_DIR%\Release" "*.out"
if defined FIRMWARE goto firmware_found
call :find_firmware "%PROJECT_DIR%\Release" "*.elf"

:firmware_found
if not defined FIRMWARE (
    echo [ERROR] No firmware output was found in Debug or Release.
    echo [ERROR] Build the CCS project first, then run Wireless Flash again.
    exit /b 5
)
if not exist "%FIRMWARE%" (
    echo [ERROR] Firmware does not exist: "%FIRMWARE%"
    exit /b 5
)

rem OpenOCD commands are parsed by Tcl, where Windows backslashes are escapes.
rem Use forward slashes and Tcl braces so firmware paths with spaces remain intact.
set "FIRMWARE_TCL=%FIRMWARE:\=/%"

if not exist "%OPENOCD_EXE%" (
    echo [INFO] Firmware: "%FIRMWARE%"
    echo [ERROR] OPENOCD_EXE does not exist: "%OPENOCD_EXE%"
    exit /b 2
)

echo [INFO] Project: "%PROJECT_DIR%"
echo [INFO] OpenOCD: "%OPENOCD_EXE%"
echo [INFO] Firmware: "%FIRMWARE%"
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
    echo [ERROR] Flash was not started.
    exit /b 4
)

echo [INFO] Adapter: CMSIS-DAP over SWD
echo [INFO] SWD speed: %SWD_SPEED_KHZ% kHz
echo [WARNING] The next command will program and verify Flash, then reset the MCU.
echo [COMMAND] "%OPENOCD_EXE%" -s "%OPENOCD_SCRIPTS%" -f interface/cmsis-dap.cfg -c "transport select swd" -c "adapter speed %SWD_SPEED_KHZ%" -f target/ti_mspm0.cfg -c "program {%FIRMWARE_TCL%} verify reset exit"
echo [INFO] Starting Wireless Flash...

"%OPENOCD_EXE%" ^
  -s "%OPENOCD_SCRIPTS%" ^
  -f interface/cmsis-dap.cfg ^
  -c "transport select swd" ^
  -c "adapter speed %SWD_SPEED_KHZ%" ^
  -f target/ti_mspm0.cfg ^
  -c "program {%FIRMWARE_TCL%} verify reset exit"

set "OPENOCD_RC=%ERRORLEVEL%"
if not "%OPENOCD_RC%"=="0" (
    echo [ERROR] Wireless Flash failed. OpenOCD exit code: %OPENOCD_RC%
    echo [HINT] Check the CMSIS-DAP connection, target power, SWD wiring, and try SWD_SPEED_KHZ=100.
    exit /b %OPENOCD_RC%
)

echo [OK] Wireless Flash completed and verified successfully.
exit /b 0

:find_firmware
if not exist "%~1\" exit /b 0
for /f "delims=" %%F in ('dir /b /a-d /o-d "%~1\%~2" 2^>nul') do if not defined FIRMWARE set "FIRMWARE=%~1\%%F"
exit /b 0
