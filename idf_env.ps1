$env:IDF_TOOLS_PATH = "C:\Espressif"

Set-Location "C:\Espressif\v5.3.5\esp-idf"
. .\export.ps1

Set-Location "C:\esp\5_ESP32P4C6_HMI_7inch"

Write-Host ""
Write-Host "ESP-IDF environment ready for 5_ESP32P4C6_HMI_7inch" -ForegroundColor Green
idf.py --version