$ErrorActionPreference = 'Stop'
$root = Split-Path -Parent $PSScriptRoot
Write-Host "Cleaning bin/obj under $root"
Get-ChildItem -Path $root -Directory -Recurse -Force |
  Where-Object { $_.Name -in @('bin','obj') } |
  Sort-Object FullName -Descending |
  ForEach-Object {
    Write-Host "Remove $($_.FullName)"
    Remove-Item -LiteralPath $_.FullName -Recurse -Force -ErrorAction SilentlyContinue
  }
Write-Host 'Done.'
