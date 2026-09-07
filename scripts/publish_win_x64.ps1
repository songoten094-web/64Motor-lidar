param(
  [string]$Configuration = "Release",
  [string]$Runtime = "win-x64"
)

$ErrorActionPreference = "Stop"
$root = Split-Path -Parent $PSScriptRoot
$project = Join-Path $root "src\LivoxHmi.App\LivoxHmi.App.csproj"
$out = Join-Path $root "publish\$Runtime"

Write-Host "Publishing Livox + WaveMotion single HMI ($Runtime)..." -ForegroundColor Cyan
dotnet publish $project -c $Configuration -r $Runtime --self-contained true -o $out
Write-Host "Publish ready: $out" -ForegroundColor Green
Write-Host "Single-HMI self-contained package ready."
