param([string]$Configuration = "Debug")
$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot

Write-Host "[1/2] Building Livox native bridge..." -ForegroundColor Cyan
& (Join-Path $PSScriptRoot "build_native.ps1") -Configuration $Configuration

Write-Host "[2/2] Building single HMI solution..." -ForegroundColor Cyan
dotnet build (Join-Path $root "LivoxWaveHmi.sln") -c $Configuration

Write-Host "Single HMI build complete." -ForegroundColor Green
