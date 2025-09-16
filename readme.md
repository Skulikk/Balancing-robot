# README

# Demonstrační video

Ve složce na Nextcloud jsou dvě videa - **demonstrace_projektu.mp4** a **demonstrace_projektu_auto.mp4**.

První jmenované ukazuje schopnost robota zdvihnout se ze země, jezdit, zatáčet a reagovat na postrčení.

Druhé video ukazuje autonomní jízdu. Jak je popsáno v bakalářské práci, kvůli zvolenému senzoru vzdálenosti není autonomní jízda dokonalá. Robot se z důvodu nekvalitních dat ze senzoru musí pohybovat velmi pomalu a několikrát zastavit a ujistit se o vzdálenosti.

# Instalace

## Požadavky

Pro použití ROS2 Humble je vyžadován systém Ubuntu 22.04. Pro správný běh programu je vhodné, aby měl operační systém kernel preempt_rt.

## Knihovny

Před kompilací programu je nutné nainstalovat následující knihovny a balíčky:

- pigpio
- RTIMULib
- gpiod
- BlueDot
- ROS2 Humble

## Kompilace

Projekt se kompiluje příkazem **colcon build**. Ten se volá v kořenové složce projektu.

# Spuštění

Robot se spustí po zapnutí napájení. Balanční program je nastaven tak, aby se automaticky spustil po startu operačního systému. Uživatele na to upozorní krátký zvukový tón. Po zaznění tónu se uživatel může připojit k robotovi pomocí mobilního telefonu s nainstalovanou aplikací Blue Dot (telefon musí být spárovaný s Raspberry).

V horním řádku úvodní obrazovky jsou kontrolky indikující stav senzorů. Pokud je některá z nich červená, je potřeba zkontrolovat správné zapojení senzorů a případně restartovat program (černé tlačítko nahoře).
Prostřední tlačítko (červená/zelená) aktivuje balancování. Tlačítko pod ním (žlutá/zelená) aktivuje autonomní mód. Spodní černé tlačítko přepíná na režim joysticku.
V režimu joysticku se robot ovládá pohybem prstu po modrém kruhu. Pro návrat na úvodní obrazovku je potřeba na joystick dvakrát poklepat.

Pokud je robot spouštěn manuálně s využitím SSH, postup je následující:

- **cd ~/robot**
- **source install/setup.bash**
- **ros2 launch robot_bringup robot_launch.py**

# Struktura

Ve stromě jsou uvedeny věechny soubory, které nebyly automaticky vygenerované systémem ROS2.

```text
└── robot
    ├── RTIMULib.ini
    ├── restart.sh
    ├── run_encoder_exc.sh
    ├── build
    ├── install
    ├── log
    └── src
        ├── bluetooth_pkg
        │   ├── bluetooth_pkg
        │   │   └── bluetooth_node.py
        │   ├── package.xml
        │   ├── setup.cfg
        │   └── setup.py
        ├── control_pkg
        │   ├── CMakeLists.txt
        │   ├── package.xml
        │   └── src
        │       └── motor_controller_node.cpp
        ├── robot_bringup
        │   ├── package.xml
        │   ├── resource
        │   │   └── robot_bringup
        │   ├── robot_bringup
        │   │   └── launch
        │   │       └── robot_launch.py
        │   ├── setup.cfg
        │   └── setup.py
        └── sensors_pkg
            ├── CMakeLists.txt
            ├── msg
            │   ├── EncoderData.msg
            │   └── IMUData.msg
            ├── package.xml
            └── src
                ├── encoder_node.cpp
                ├── imu_node.cpp
                └── ultra_s_node.cpp
```