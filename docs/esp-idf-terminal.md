# ESP-IDF Terminal Setup (VS Code) – zuverlässige Nutzung

## 🎯 Ziel

Dieses Dokument stellt sicher, dass das ESP-IDF-Projekt **immer mit der richtigen Umgebung** gebaut und geflasht wird.

Hintergrund:
Die von Espressif erzeugte Datei
`C:\Espressif\tools\Microsoft.v5.3.5.PowerShell_profile.ps1`
ist aktuell fehlerhaft und darf **nicht verwendet werden**.

---

## ✅ Empfohlener Weg (Standard)

### 1. VS Code öffnen

Projektordner öffnen:

```
C:\esp\5_ESP32P4C6_HMI_7inch
```

---

### 2. Richtiges Terminal starten

**Wichtig: NICHT verwenden:**

* ❌ ESP-IDF Terminal (Extension)
* ❌ Standard PowerShell ohne Setup

**Richtig:**

```
Terminal → Pfeil neben "+"
→ "ESP-IDF PowerShell"
```

---

### 3. Erwartete Ausgabe

Nach dem Start muss folgendes erscheinen:

```
ESP-IDF environment ready for 5_ESP32P4C6_HMI_7inch
ESP-IDF v5.3.5
```

Wenn das nicht erscheint → falsches Terminal

---

### 4. Befehle

```powershell
idf.py build
idf.py -p COM3 flash monitor
```

---

## 🔁 Alternative (manuell starten)

Falls falsches Terminal offen ist:

```powershell
cd C:\esp\5_ESP32P4C6_HMI_7inch
Set-ExecutionPolicy -Scope Process Bypass
.\idf_env.ps1
```

---

## ❌ Was NICHT benutzt werden darf

### ESP-IDF Button in VS Code

```
Build Project
Flash
Monitor
```

→ nutzt kaputtes Profil → führt zu Fehlern

---

### ESP-IDF Terminal (Extension)

```
ESP-IDF: Open Terminal
```

→ nutzt:

```
Microsoft.v5.3.5.PowerShell_profile.ps1
```

→ ist defekt → nicht verwenden

---

## 🧠 Erkennungsregeln

| Zustand                   | Bedeutung              |
| ------------------------- | ---------------------- |
| `idf.py not recognized`   | Umgebung nicht geladen |
| ParserError PowerShell    | falsches Profil        |
| `ESP-IDF v5.3.5` sichtbar | ✅ korrekt              |

---

## 🚀 Empfehlung Workflow

1. VS Code öffnen
2. Terminal schließen
3. **ESP-IDF PowerShell öffnen**
4. `idf.py build`
5. `idf.py flash monitor`

---

## 📌 Optional (Absicherung)

Fallback über CMD:

```cmd
idf_env.cmd
idf.py build
```

---

## ✅ Status dieses Projekts

* ESP-IDF Version: **v5.3.5**
* Ziel: **ESP32-P4**
* Display + LVGL: ✔
* UART ETH↔HMI: ✔
* State Cache: ✔

---

## 🔜 Zukunft

Wenn Espressif das Profil fixt, kann dieses Dokument vereinfacht werden.

Bis dahin gilt:

👉 **Nur ESP-IDF PowerShell verwenden**
