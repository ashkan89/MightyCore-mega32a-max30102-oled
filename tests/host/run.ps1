# ------------------------------------------------------------------
#  Build and run the host-side DSP tests.
#
#  These compile the real src/ppg.c and src/sys.c against the stub AVR
#  headers in tests/host/stub, so what is tested is the firmware's own
#  code rather than a copy of it.  Nothing here touches hardware; see the
#  header of test_ppg.c for what that does and does not prove.
#
#  Usage, from the repository root:
#      powershell -ExecutionPolicy Bypass -File tests\host\run.ps1
#
#  On a machine with gcc or clang instead of MSVC, this is equivalent:
#      cc -I tests/host/stub -I src -o test_ppg tests/host/test_ppg.c -lm
#      ./test_ppg
# ------------------------------------------------------------------
$ErrorActionPreference = 'Stop'

$root = Split-Path -Parent (Split-Path -Parent $PSScriptRoot)
$out  = Join-Path $env:TEMP 'pulseox_tests'
New-Item -ItemType Directory -Force -Path $out | Out-Null

# --- locate a compiler -------------------------------------------------
$cl = $null
$vswhere = Join-Path ${env:ProgramFiles(x86)} 'Microsoft Visual Studio\Installer\vswhere.exe'
if (Test-Path $vswhere) {
    $vsdir = & $vswhere -latest -property installationPath
    if ($vsdir) {
        # Hostx64\x64 cl.exe, whichever MSVC toolset version is installed.
        $cl = Get-ChildItem -Path (Join-Path $vsdir 'VC\Tools\MSVC') -Recurse `
                -Filter 'cl.exe' -ErrorAction SilentlyContinue |
              Where-Object { $_.FullName -like '*Hostx64\x64\cl.exe' } |
              Select-Object -First 1 -ExpandProperty FullName
    }
}

if (-not $cl) {
    Write-Host "No MSVC found. With gcc or clang available, run instead:"
    Write-Host "  cc -I tests/host/stub -I src -o test_ppg tests/host/test_ppg.c -lm"
    exit 1
}

# cl.exe needs its own include/lib environment.  Rather than invoking
# VsDevCmd.bat -- which shells out and is awkward to capture -- point the
# compiler at the toolset's own headers and the Windows SDK directly.
$msvcRoot = Split-Path -Parent (Split-Path -Parent (Split-Path -Parent (Split-Path -Parent $cl)))
$sdkRoot  = 'C:\Program Files (x86)\Windows Kits\10'
$sdkVer   = (Get-ChildItem (Join-Path $sdkRoot 'Include') -ErrorAction SilentlyContinue |
             Sort-Object Name -Descending | Select-Object -First 1).Name

$inc = @(
    (Join-Path $msvcRoot 'include')
    (Join-Path $sdkRoot "Include\$sdkVer\ucrt")
    (Join-Path $sdkRoot "Include\$sdkVer\shared")
    (Join-Path $sdkRoot "Include\$sdkVer\um")
)
$lib = @(
    (Join-Path $msvcRoot 'lib\x64')
    (Join-Path $sdkRoot "Lib\$sdkVer\ucrt\x64")
    (Join-Path $sdkRoot "Lib\$sdkVer\um\x64")
)

$env:INCLUDE = ($inc -join ';')
$env:LIB     = ($lib -join ';')
$env:Path    = (Split-Path -Parent $cl) + ';' + $env:Path

# --- compile -----------------------------------------------------------
# /TC  compile as C, not C++
# /W3  MSVC's own warnings.  The AVR build is the one held to a clean
#      -Wall -Wextra bar; running the logic through a second compiler with
#      different type widths is worth something on its own, because it
#      catches anything that silently depended on a 16-bit int.
# /wd4244 /wd4267  narrowing conversions, deliberate and explicit
#      throughout the fixed-point code
$exe = Join-Path $out 'test_ppg.exe'
& $cl /nologo /TC /W3 /Od `
    /wd4244 /wd4267 /wd4146 `
    "/I$(Join-Path $root 'tests\host\stub')" "/I$(Join-Path $root 'src')" `
    "/Fo$out\" "/Fe$exe" `
    (Join-Path $root 'tests\host\test_ppg.c')
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }

# --- run ---------------------------------------------------------------
& $exe
exit $LASTEXITCODE
