@echo off
rem ------------------------------------------------------------------
rem  Build and run the host-side DSP tests with MSVC.
rem
rem  These compile the real src/ppg.c and src/sys.c against the stub AVR
rem  headers in tests/host/stub, so what is tested is the firmware's own
rem  code rather than a copy of it.  Nothing here touches hardware; see
rem  the header of test_ppg.c for what that does and does not prove.
rem
rem  Usage:  tests\host\run.cmd
rem ------------------------------------------------------------------
setlocal EnableDelayedExpansion

rem %ProgramFiles(x86)% cannot be expanded inside a for/if block -- the
rem parentheses in the name terminate the block -- so copy it out first.
set "PFX86=%ProgramFiles(x86)%"
set "VSWHERE=%PFX86%\Microsoft Visual Studio\Installer\vswhere.exe"

set "VSDIR="
if exist "%VSWHERE%" (
  for /f "usebackq delims=" %%i in (`"!VSWHERE!" -latest -property installationPath`) do set "VSDIR=%%i"
)

if not defined VSDIR (
  echo Could not locate Visual Studio.
  echo.
  echo Install the "Desktop development with C++" workload, or build the
  echo tests with any host C compiler:
  echo.
  echo   cc -I tests/host/stub -I src -o test_ppg tests/host/test_ppg.c -lm
  echo   ./test_ppg
  exit /b 1
)

rem VsDevCmd.bat rather than vcvars64.bat: in Visual Studio 18 the latter
rem delegates to a vcvarsall.bat that is no longer shipped.
call "%VSDIR%\Common7\Tools\VsDevCmd.bat" -arch=x64 -host_arch=x64 -no_logo
if errorlevel 1 exit /b 1

set "OUT=%TEMP%\pulseox_tests"
if not exist "%OUT%" mkdir "%OUT%"

rem /TC     compile as C, not C++
rem /W3     MSVC's own warnings.  The AVR build is the one held to
rem         -Wall -Wextra and a clean-warnings bar; this is only here to
rem         run the logic, on a compiler with different type widths --
rem         which is itself worth something, since it catches anything
rem         that silently depended on 16-bit int.
rem /wd4244 /wd4267  narrowing conversions, deliberate throughout the
rem         fixed-point code and already explicit in the source
cl /nologo /TC /W3 /Od /Zi ^
   /wd4244 /wd4267 /wd4146 ^
   /I tests\host\stub /I src ^
   /Fo"%OUT%\\" /Fd"%OUT%\\" /Fe"%OUT%\test_ppg.exe" ^
   tests\host\test_ppg.c
if errorlevel 1 exit /b 1

"%OUT%\test_ppg.exe"
exit /b %ERRORLEVEL%
