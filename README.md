# ESP32-P4 HMI 7inch

Migrationstest für das Eisenbahn-HMI auf Waveshare ESP32-P4-WIFI6-Touch-LCD-7B.

## Stand

- ESP-IDF v5.3.5
- LVGL v8
- Display und Touch laufen
- UART1 ETH <-> HMI läuft mit 230400 Baud
- P4 TX = GPIO34
- P4 RX = GPIO36
- state-lite und analog werden empfangen und gezählt
- ACKs werden gesendet

## Ziel

Schrittweise Migration des bisherigen ESP32-S3-HMI auf die neue ESP32-P4-Hardware.