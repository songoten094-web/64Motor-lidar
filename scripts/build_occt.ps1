param([ValidateSet('Debug','Release')][string]$Configuration='Debug')
$ErrorActionPreference='Stop'
$root = Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)
$src = Join-Path $root 'native_occt'
$build = Join-Path $src 'build-win-x64'
if (-not (Get-Command cmake -ErrorAction SilentlyContinue)) { throw 'CMake not found in PATH.' }
$args = @('-S',$src,'-B',$build,'-A','x64')
if ($env:VCPKG_ROOT) { $toolchain = Join-Path $env:VCPKG_ROOT 'scripts\buildsystems\vcpkg.cmake'; if (Test-Path $toolchain) { $args += "-DCMAKE_TOOLCHAIN_FILE=$toolchain"; $args += '-DVCPKG_TARGET_TRIPLET=x64-windows' } }
if ($env:OpenCASCADE_DIR) { $args += "-DOpenCASCADE_DIR=$env:OpenCASCADE_DIR" }
elseif ($env:OCCT_ROOT) {
  $cmakeDir = Join-Path $env:OCCT_ROOT 'lib\cmake\opencascade'
  if (Test-Path $cmakeDir) { $args += "-DOpenCASCADE_DIR=$cmakeDir" }
}
& cmake @args
& cmake --build $build --config $Configuration --target OcctHmiBridge
if ($LASTEXITCODE -ne 0) { throw 'OCCT bridge build failed.' }
$bin = Join-Path $build "bin\$Configuration"
Write-Host "OCCT bridge: $bin\OcctHmiBridge.dll"
# Copy OCCT runtime DLLs next to bridge when OCCT_ROOT is provided.
if ($env:VCPKG_ROOT) {
  $vcpkgBin = Join-Path $env:VCPKG_ROOT 'installed\x64-windows\bin'
  if (Test-Path $vcpkgBin) { Copy-Item (Join-Path $vcpkgBin '*.dll') $bin -Force -ErrorAction SilentlyContinue }
}
if ($env:OCCT_ROOT) {
  $occtBin = Join-Path $env:OCCT_ROOT 'win64\vc14\bin'
  if (-not (Test-Path $occtBin)) { $occtBin = Join-Path $env:OCCT_ROOT 'bin' }
  if (Test-Path $occtBin) { Copy-Item (Join-Path $occtBin '*.dll') $bin -Force -ErrorAction SilentlyContinue }
}
