#!/usr/bin/env bash
# Eris BLE — Release APK Build
# Run from the app root: bash scripts/build-release-apk.sh
# Same steps and output as robiolab_inventory_app/scripts/build-release-apk.sh.
set -euo pipefail

echo "=== Eris BLE Release APK Build ==="
echo ""

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

# 1. Install dependencies (does not refresh @robiolab/native-ui; see `npm run ui:update`)
echo "[1/5] Installing dependencies..."
npm install

# 2. Clean prebuild and regenerate android/ from current assets + app.json
echo "[2/5] Running expo prebuild --clean..."
npx expo prebuild --clean --platform android

# 3. Build release APK
echo "[3/5] Building release APK..."
cd android
./gradlew assembleRelease
cd "$PROJECT_ROOT"

# 4. Locate the APK
APK_PATH="android/app/build/outputs/apk/release/app-release.apk"

if [ -f "$APK_PATH" ]; then
  APK_SIZE=$(du -h "$APK_PATH" | cut -f1)
  echo ""
  echo "=== Build Successful ==="
  echo "APK: $APK_PATH"
  echo "Size: $APK_SIZE"
  echo ""
else
  echo ""
  echo "=== Build Failed ==="
  echo "APK not found at $APK_PATH"
  exit 1
fi
