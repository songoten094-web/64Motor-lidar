param(
    [ValidateSet("Debug","Release","RelWithDebInfo","MinSizeRel")]
    [string]$Configuration = "Debug"
)

$ErrorActionPreference = "Stop"

$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$native = Join-Path $root "native"
$build = Join-Path $native "build-win-x64"

Write-Host "=== Livox MID-360 native bridge / Windows x64 ===" -ForegroundColor Cyan
Write-Host "Native source : $native"
Write-Host "Build folder  : $build"
Write-Host "Configuration  : $Configuration"

if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) {
    throw "CMake was not found in PATH. Install/enable CMake from Visual Studio Installer or add cmake.exe to PATH."
}

# A fresh Windows build directory is intentional. Do not reuse the old
# native/build directory that may have been generated on Linux/WSL.
if (-not (Test-Path (Join-Path $build "CMakeCache.txt"))) {
    cmake -S $native -B $build -G "Visual Studio 17 2022" -A x64
}

cmake --build $build --config $Configuration --target LivoxHmiBridge

$dll = Join-Path $build "bin\$Configuration\LivoxHmiBridge.dll"
if (-not (Test-Path $dll)) {
    throw "Native build finished but LivoxHmiBridge.dll was not produced: $dll"
}

Write-Host ""
Write-Host "[OK] LivoxHmiBridge.dll" -ForegroundColor Green
Write-Host "     $dll"
Write-Host ""
Write-Host "Now build LivoxHmiV1.sln in the same configuration." -ForegroundColor Yellow
Write-Host "The .NET project will copy LivoxHmiBridge.dll beside the EXE automatically."
