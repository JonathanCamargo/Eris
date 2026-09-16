# Eris BLE — Release APK Build
# Run from the app root: .\scripts\build-release-apk.ps1
# Same steps and output as robiolab_inventory_app/scripts/build-release-apk.ps1.

$ErrorActionPreference = "Stop"

Write-Host "=== Eris BLE Release APK Build ===" -ForegroundColor Cyan
Write-Host ""

$ProjectRoot = Resolve-Path (Join-Path $PSScriptRoot "..")
Set-Location $ProjectRoot

# 1. Install dependencies (does not refresh @robiolab/native-ui; see `npm run ui:update`)
Write-Host "[1/5] Installing dependencies..." -ForegroundColor Yellow
npm install
if ($LASTEXITCODE -ne 0) { throw "npm install failed" }

# 2. Clean prebuild and regenerate android/ from current assets + app.json
Write-Host "[2/5] Running expo prebuild --clean..." -ForegroundColor Yellow
npx expo prebuild --clean --platform android
if ($LASTEXITCODE -ne 0) { throw "expo prebuild failed" }

# 3. Build release APK
Write-Host "[3/5] Building release APK..." -ForegroundColor Yellow
Set-Location android
.\gradlew.bat assembleRelease
if ($LASTEXITCODE -ne 0) { throw "gradlew assembleRelease failed" }
Set-Location $ProjectRoot

# 4. Locate the APK
$ApkPath = Join-Path $ProjectRoot "android\app\build\outputs\apk\release\app-release.apk"

if (Test-Path $ApkPath) {
    $ApkSize = (Get-Item $ApkPath).Length
    $ApkSizeMB = [math]::Round($ApkSize / 1MB, 2)
    Write-Host ""
    Write-Host "=== Build Successful ===" -ForegroundColor Green
    Write-Host "APK:  $ApkPath"
    Write-Host "Size: $ApkSizeMB MB"
    Write-Host ""
} else {
    Write-Host ""
    Write-Host "=== Build Failed ===" -ForegroundColor Red
    Write-Host "APK not found at $ApkPath"
    exit 1
}
