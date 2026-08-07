@echo off
setlocal EnableExtensions DisableDelayedExpansion

for %%I in ("%~dp0..") do set "PROJECT_ROOT=%%~fI"
set "BUILD_DIR=%PROJECT_ROOT%\Debug"

if defined CCS_GMAKE_EXE (
    set "GMAKE_EXE=%CCS_GMAKE_EXE%"
) else (
    set "GMAKE_EXE=D:\ti\ccstheia151\ccs\utils\bin\gmake.exe"
)

echo [Wireless] Project: "%PROJECT_ROOT%"
echo [Wireless] Builder: "%GMAKE_EXE%"

if not exist "%GMAKE_EXE%" (
    echo [ERROR] CCS gmake was not found: "%GMAKE_EXE%"
    exit /b 2
)

if not exist "%BUILD_DIR%\makefile" (
    echo [ERROR] CCS generated makefile was not found: "%BUILD_DIR%\makefile"
    exit /b 3
)

pushd "%BUILD_DIR%" >nul
"%GMAKE_EXE%" all
set "BUILD_RC=%ERRORLEVEL%"
popd >nul

if not "%BUILD_RC%"=="0" (
    echo [ERROR] Build failed with exit code %BUILD_RC%. Wireless Flash was not started.
    exit /b %BUILD_RC%
)

echo [Wireless] Build succeeded. Starting Wireless Flash...
call "%PROJECT_ROOT%\Scripts\wireless_flash.bat"
set "FLASH_RC=%ERRORLEVEL%"

if not "%FLASH_RC%"=="0" (
    echo [ERROR] Wireless Flash failed with exit code %FLASH_RC%.
    exit /b %FLASH_RC%
)

echo [Wireless] Build + Wireless Flash completed successfully.
exit /b 0
