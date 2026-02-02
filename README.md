# Avular S7 Loader Demonstrator

Welkom bij de software repository van de **Avular Loader Demonstrator**. 
Dit project bestuurt een autonome 'skid-steer' loader op schaal. Het systeem maakt gebruik van een Ubuntu Mini PC (High-Level Control) en een Teensy 4.1 (Low-Level Control), die communiceren via ROS 2 Jazzy en Micro-ROS.

---

## Opstartprocedure

Het systeem is geconfigureerd om volledig automatisch op te starten zodra de PC stroom krijgt. Een terminalvenster opent vanzelf, reset de Teensy, en start de software.

### Handmatig Starten (Command Line)
Mocht de automatische start niet werken, of wil je het systeem handmatig starten voor debugging:

1.  Sluit de USB-kabel aan tussen de PC en de Teensy.
2.  Open een terminal en voer het volgende commando uit:
    ```bash
    ros2 launch loader_control avuloader_startup_script.py
    ```
    *Dit script reset automatisch de Teensy (om de verbinding te forceren), wacht 3 seconden, en start daarna de Micro-ROS Agent en alle control nodes.*

---

## Veilig Afsluiten (Shutdown)
Om corruptie van de PC te voorkomen, moet het systeem netjes worden afgesloten voordat de hoofdstroomschakelaar wordt omgezet.

1.  Houd op de PS4 Controller de knoppen **SHARE** en **OPTIONS** tegelijk ingedrukt.
2.  De LED-strips op de loader worden **Solid Magenta (Paars)**.
3.  Wacht 20 seconden.
4.  Zet de hoofdschakelaar om.

---

## Besturing (PS4 Controller)
De robot wordt bestuurd met een Sony DualShock 4 controller via bluethooth.

### Rijden & Armen
| Input | Functie | Beschrijving |
| :--- | :--- | :--- |
| **Linker Joystick** | Rijden (Skid Steer) | Omhoog/Omlaag = Gas, Links/Rechts = Draaien |
| **Rechter Joystick** | Hefarm & Bak | Omhoog/Omlaag = Arm, Links/Rechts = Bak Kantelen |

### Knoppen & Presets
| Knop | Functie | Actie |
| :--- | :--- | :--- |
| **Kruisje (X)** | Preset: Rij-stand | Arm laag, bak ingetrokken (Veilig rijden) |
| **Rondje (O)** | Preset: Transport | Arm op halfhoogte (Voor laden/lossen) |
| **Driehoekje (△)** | Preset: Dump Hoog | Arm maximaal omhoog, bak in kiepstand |
| **Share + Options** | **SHUTDOWN** | Sluit de Linux PC veilig af (LEDs -> Paars) |

---

## LED Status Indicatoren
De LED-strips geven visuele feedback over de staat van de robot en veiligheid.

| Kleur / Effect | Status | Betekenis voor Operator |
| :--- | :--- | :--- |
| **Blauw (Knipperend)** | Startup | Systeem is aan het opstarten. Wacht even. |
| **Blauw (Ademend)** | Standby | Systeem is klaar en verbonden, maar motoren staan stil. |
| **Groen (Vast)** | Driving | Robot is in beweging (**Pas op!**). |
| **Groen (Ademend)** | Operational | Robot is actief, maar staat momenteel stil. |
| **Rood (Vast)** | E_Brake | Noodstop is ingedrukt of software blokkade. |
| **Geel (Vast)** | Low_power | Accu spanning is laag (< 19V). Opladen nodig. |
| **Magenta (Vast)** | Shutdown | **VEILIG:** PC is uitgeschakeld. Stroom mag eraf. |
| **Rood/Oranje (Knipperend)**| Battery_empty | Kritiek accuniveau. Systeem schakelt uit. |

---

## Development Handleiding
De codebase is gesplitst in twee delen. Hieronder staat hoe je wijzigingen compileert en uploadt.

### 1. High-Level Software (ROS 2 / Python)
**Locatie:** `~/Avular-S7/PC/src/loader_control`

Als je wijzigingen aanbrengt in de Python nodes (bijv. knoppen mapping, snelheden):

1.  Ga naar de workspace:
    ```bash
    cd ~/Avular-S7/PC/joy_load3r
    ```
2.  Build de package (gebruik symlink zodat je niet steeds opnieuw hoeft te builden bij kleine Python wijzigingen):
    ```bash
    colcon build --symlink-install
    ```
3.  Ververs je omgeving:
    ```bash
    source install/setup.bash
    ```
4.  Herstart de node uit Stap 3.

### 2. Low-Level Firmware (C++ / Teensy)

**Locatie:** `~/Avular-S7/PC/src/firmware` (PlatformIO Project)

Als je wijzigingen aanbrengt in de C++ code:

1. BELANGRIJK: Stop eerst alle ROS processen (sluit de terminal met de loader software). Reden: De Micro-ROS Agent houdt de USB-poort bezet. Uploaden mislukt als deze nog draait.

2. Open de map in VS Code.

3. Klik op het PlatformIO Upload Icoon (Pijltje onder in de balk).

4. Wacht op de melding `[SUCCESS]`.

5. Start de ROS software opnieuw op.

## Troubleshooting

- De Teensy maakt geen verbinding ("Session not established").

    - De automatische reset heeft mogelijk gefaald. Druk 1x kort op de fysieke RESET knop op de Teensy terwijl de ROS Agent in de terminal draait.

- De controller reageert niet.
    - Controleer of de controller aan staat (brandend licht).
    - Check of de Linux PC de controller ziet via:
      ```bash
    ros2 run joy joy_node
    ```
