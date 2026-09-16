# Eris BLE — Release APK Build

Identical process to the other Robiolab apps (`robiolab_inventory_app/scripts`).
Run from the app root (`demos/android_ble/app`):

**PowerShell (Windows):**
```powershell
.\scripts\build-release-apk.ps1
```

**Bash (macOS/Linux):**
```bash
bash scripts/build-release-apk.sh
```

What it does:
1. `npm install` — ensures deps are up to date. It does **not** refresh the `@robiolab/native-ui`
   copy: after changing `robiolab_native_ui`, bump its version and run `npm run ui:update` first
2. `npx expo prebuild --clean` — regenerates `android/` from current `app.json` and the brand assets in `@robiolab/native-ui`
3. `./gradlew assembleRelease` — builds the release APK
4. Prints the APK path and size

Output APK: `android/app/build/outputs/apk/release/app-release.apk`

Needs `C:\git\robiolab\robiolab_native_ui` next to the Eris checkout (see `@robiolab/native-ui` in
`package.json`) until that package is published to its own git repository.
