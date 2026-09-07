$ErrorActionPreference = "Stop"
$root = Resolve-Path (Join-Path $PSScriptRoot "..")
$build = Join-Path $root "native\build-win-x64"
if (Test-Path $build) {
    Remove-Item $build -Recurse -Force
    Write-Host "Removed $build"
} else {
    Write-Host "Nothing to clean."
}
