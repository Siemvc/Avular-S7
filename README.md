# Avular S7 Loader 

Deze handleiding beschrijft hoe je het ROS 2 systeem opstart om de loader aan te sturen met een PS4 controller via de Teensy 4.1.

## Opstartprocedure

Voor een volledig werkend systeem moeten er 4 processen tegelijk draaien. Open hiervoor 4 tabbladen of vensters in je terminal.

### Stap 1: De Brug (Micro-ROS Agent)
Zorgt voor de communicatie tussen de PC en de Teensy.

1.  Sluit de Teensy aan via USB.
2.  Voer de volgende commando's uit:
    ```bash
    source ~/microros_ws/install/setup.bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
    ```
3.  **BELANGRIJK:** Druk **1x op de RESET knop** op de Teensy.
4.  Wacht tot je de melding `Session established` ziet.

### Stap 2: De Input (PS4 Driver)
Leest de ruwe data van de controller uit.

1.  Zorg dat de PS4 controller verbonden is (Bluetooth of Kabel).
2.  Open een **nieuwe terminal** en voer uit:
    ```bash
    ros2 run joy joy_node
    ```

### Stap 3: De Vertaler (Control Node)
Vertaalt joystick signalen naar commando's voor de loader.

1.  Open een **nieuwe terminal**.
2.  Navigeer naar de workspace en start de node:
    ```bash
    cd ~/Avular-S7/PC/joy_load3r
    source install/setup.bash
    ros2 run control_turtle control_Node
    ```

### Stap 4: Feedback & Debugging (Optioneel)
Controleer wat de Teensy terugstuurt (bijv. CAN data of motor status).

1.  Open een **nieuwe terminal** en voer uit:
    ```bash
    ros2 topic echo /teensy_debug
    ```
    > *Tip: Wil je andere data zien? Typ `ros2 topic list` voor een overzicht.*

---

## Development & Builden

Als je wijzigingen hebt aangebracht in de code, moet je deze opnieuw "bouwen". Dit verschilt voor de PC en de Teensy.

### A. Code op de PC gewijzigd (Python)
Als je aanpassingen hebt gedaan in de `joy_load3r` map (bijv. knoppen mapping):

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

### B. Code op de Teensy gewijzigd (C++)
Als je aanpassingen hebt gedaan in `main.cpp` (bijv. motor settings):

1.  **STOP Stap 1 (Micro-ROS Agent)** met `Ctrl + C`.
    * *Let op: Je kunt niet uploaden als de agent draait, want de USB-poort is dan bezet!*
2.  Druk in VS Code op **Upload** (Pijltje naar rechts in de blauwe balk).
3.  Wacht op "SUCCESS".
4.  Start Stap 1 (de Agent) opnieuw en druk op de reset knop van de Teensy.