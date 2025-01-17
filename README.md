# Robotik_Projekt
Turtlebot Projekt für Robotik

Zeitplan in Exel datei

## Inhaltsverzeichnis
- [Über das Projekt](#über-das-projekt)
- [Erste Schritte](#erste-schritte)
- [Voraussetzungen](#voraussetzungen)
- [Installation](#installation)
- [Verwendung](#verwendung)
- [Projektstruktur](#projektstruktur)
- [Mitwirkende](#mitwirkende)
- [Lizenz](#lizenz)

## Über das Projekt
Dieses Projekt verwendet den Turtlebot 3 Burger, um einen Linienfolger zu entwickeln, der in der Mitte von zwei Linien fahren kann, auch wenn eine oder beide Linien zwischendurch unterbrochen sind und Hindernissen auf der Strecke ausweichen kann.

## Rahmenbedingungen
- Ein Projekt pro Gruppe zu maximal 3 Personen
- 8 Seiten Bericht pro Person (z. B. bei 3 Personen 24 Seiten exklusive Anhang)
- Schriftgröße: 11pt; Zeilenabstand: 1.5
- Abgabe des Berichtes am 24.2.2024
- Praktische Vorführung des Projekts am: 6.3.2025
- Programmiersprache: C++
- Umgebung & Compiler: ROS in Linux Ubuntu LTS, catkin make

## Aufgabenstellung
1. Kalibrieren der intrinsischen und extrinsischen Kameraparameter der RGB-Kamera mithilfe eines Kalibriermusters.
2. Publizieren des rektifizierten (kalibrierten) Bildes auf einem ROS-Topic (z. B. /camera/rectified_image).
3. Bestimmen der Transformationsmatrix für die perspektivische Transformation (z. B. Homographie-Matrix).
4. Publizieren des transformierten Bildes in der Draufsicht (Bird’s-Eye-View) auf einem ROS-Topic (z. B. /camera/birdseye_image).
5. Bestimmen der Transformationsmatrix zwischen der Draufsicht und der Punktwolke des Laserscanners.
6. Publizieren des Draufsicht-Bildes inklusive der eingezeichneten Laserpunkte als Overlay (z. B. /camera/birdseye_with_lidar).
7. Implementieren eines Algorithmus zum Erkennen und Folgen einer durch zwei Linien begrenzten Fahrbahn. Der Roboter soll dabei mittig der Linien fahren und beim Ausbleiben einer der Liniensegmente interpolieren.

### Zusatzaufgaben:
1. Hinderniserkennung und -vermeidung (5 Sonderpunkte)
2. Darstellen von Trajektorie und Hindernissen in RViz (5 Sonderpunkte)
3. Bilddaten im Stitching-Verfahren als Karte darstellen (10 Sonderpunkte)

## Erste Schritte
Um eine lokale Kopie des Projekts zu erhalten und es auszuführen, folgen Sie diesen einfachen Schritten.

## Voraussetzungen
- ROS (Robot Operating System)
- Turtlebot 3 Burger
- C++
- Weitere Abhängigkeiten (siehe `requirements.txt`)

## Installation
1. Klonen Sie das Repository:
    ```bash
    git clone https://github.com/username/Robotik_Projekt.git
    ```
2. Installieren Sie die erforderlichen Abhängigkeiten:
    ```bash
    catkin_make
    ```

## Verwendung
Beschreiben Sie hier, wie das Projekt verwendet werden kann. Fügen Sie Beispiele und Anweisungen hinzu.

## Projektstruktur
Beschreiben Sie die Struktur des Projekts, z.B.:
```
├── src
│   ├── main.cpp
│   ├── line_follower.cpp
│   └── obstacle_avoidance.cpp
├── README.md
└── requirements.txt
```

## Mitwirkende
- Niklas von Seggern
- Janis Schneider

## Lizenz
Dieses Projekt ist unter der MIT-Lizenz lizenziert – siehe die [LICENSE](LICENSE) Datei für Details.
