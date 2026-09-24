# GELLO Panda LeRobot Teleoperation

This repository contains my modified GELLO setup for teleoperating a Franka Emika Panda robot and recording demonstrations for imitation learning with LeRobot.

The project is based on the original GELLO software and was extended for my own Franka Panda, Polymetis, camera recording, and LeRobot dataset conversion pipeline.

## License and Attribution

This repository is based on the original GELLO software by Philipp Wu.

Original project: https://github.com/wuphilipp/gello_software

The original copyright and license notice are preserved in the `LICENSE` file.

This version contains project-specific modifications for Franka Emika Panda teleoperation, Polymetis-based robot control, RealSense/FRAMOS camera integration, and LeRobot dataset conversion.



## Overview

The goal of this project is to use a GELLO teleoperation device to control a Franka Emika Panda robot and record demonstration data that can later be used for imitation learning.

This repository includes modifications for:

- Franka Emika Panda teleoperation
- Polymetis-based robot control
- GELLO joint calibration and control
- RealSense / FRAMOS camera integration
- multi-camera recording support
- conversion of recorded GELLO episodes into the LeRobot dataset format
- custom start and stop scripts for the local setup

## System Setup

The current setup is mainly built around:

- Franka Emika Panda robot
- GELLO teleoperation device
- Polymetis robot interface
- RealSense / FRAMOS camera
- LeRobot dataset format
- Python 3.8 / 3.10 environments depending on the used part of the pipeline
- Ubuntu 22.04 based development machine 

##  Usage of the repository 

  ## Setup of the dependencies
  
  1. Install Gello dependencies 
  Original project: [GELLO software](https://github.com/wuphilipp/gello_software)

  2. Install polymetis dependencies 
  [Polymetis installation guide](https://facebookresearch.github.io/fairo/polymetis/installation.html)

  3. Install Intel dependencies
  Note: This is not necessarily based on a specific model of a Intel camera, but in case of different cameras, the models in the files have to be changed!

  4. Install lerobot dependencies:
  [LeRobot installation guide](https://huggingface.co/docs/lerobot/en/installation)
  Note: Lerobot training is not included in this repository. The training is executed seperate. 
  This code just contains the converter to get your pkl data from the gello into the Lerobot_v3 dataformat.



## GELLO kalibrieren

Vor der ersten Benutzung oder nach einer mechanischen Aenderung wird GELLO in
die Initialisierungspose aus dem offiziellen GELLO-Projekt gebracht. Danach auf
dem Franka-Laptop ausfuehren:

```bash
cd ~/gello_software
./calibrate_gello.sh
```

Das Skript ermittelt die Gelenk- und Greiferwerte und traegt sie automatisch in
den Panda-Block von `gello/agents/gello_agent.py` ein. Falls das automatische
Eintragen nicht funktioniert, koennen die ausgegebenen Offsets dort im Block
`# Panda / Franka` ab Zeile 102 manuell unter `joint_offsets` eingetragen
werden. Dezimalwerte sind erlaubt.

## GELLO und Franka starten

Polymetis muss auf dem Franka-Laptop installiert sein. Die USB-Base-Kamera
(D455) bleibt zu Beginn eingesteckt.

### 1. tmux-Session vorbereiten

```bash
cd ~/gello_software
./start_gello_panda.sh
```

Das Skript erzeugt sechs tmux-Fenster und schreibt den jeweils passenden Befehl
in die Kommandozeile. Die Befehle werden aus Sicherheitsgruenden noch nicht
ausgefuehrt. Zwischen den Fenstern kann mit `Ctrl-b` und anschliessend der
Fensternummer gewechselt werden.

### 2. Roboterserver starten

In dieser Reihenfolge in das jeweilige Fenster wechseln und den vorbereiteten
Befehl mit **Enter** ausfuehren. Vor dem naechsten Schritt warten, bis der
vorherige Server erfolgreich gestartet ist:

1. Fenster **0 – `robot`**
2. Fenster **1 – `gripper`**
3. Fenster **2 – `nodes`**

### 3. Kameraserver starten

1. Die USB-Base-Kamera (D455) ausstecken.
2. Fenster **4 – `camera_wrist`** oeffnen und den Befehl mit **Enter** starten.
3. Warten, bis die Wrist-Kamera erfolgreich geoeffnet wurde.
4. Die USB-Base-Kamera wieder einstecken und kurz warten, bis sie erkannt wird.
5. Fenster **3 – `camera_base`** oeffnen und den Befehl mit **Enter** starten.

Die Kameras stellen ihre Bilder auf den ZMQ-Ports `5000` (Wrist) und `5001`
(Base) bereit.

### 4. Teleoperation starten

1. GELLO vorsichtig in eine Gelenkstellung bringen, die zur aktuellen Stellung
   des realen Panda passt.
2. Fenster **5 – `env`** oeffnen.
3. Erst jetzt den vorbereiteten Befehl mit **Enter** ausfuehren.

Im geoeffneten Aufnahmefenster gelten folgende Tasten:

* **`s`** startet eine neue Aufnahme.
* **`q`** beendet die aktuelle Aufnahme.

Die Roboterbewegung laeuft auch ohne aktive Aufnahme weiter.

### 5. SSH-Bruecke fuer Aufnahmen auf dem HPC

Wenn die Aufzeichnung auf dem HPC erfolgt, muss zusaetzlich ein separates
Terminal auf dem Franka-Laptop geoeffnet bleiben. Nachdem die Fenster 0 bis 4
erfolgreich laufen, dort ausfuehren:

```bash
cd ~/gello_software
./start_franka_to_hpc_reverse_tunnel.sh
```

Die Bruecke leitet die Ports `6001`, `5000` und `5001` an den HPC weiter und
muss waehrend der gesamten Aufnahme geoeffnet bleiben. Auf dem HPC muss
zusaetzlich der passende Recorder laufen.

## Beenden

Zum Beenden der Laptop-Prozesse kann in einem separaten Terminal ausgefuehrt
werden:

```bash
cd ~/gello_software
./stop_gello_panda.sh
```

Eine genauere Liste der fuer den Franka-Laptop benoetigten Dateien steht in
[`docs/franka_laptop_startup.md`](docs/franka_laptop_startup.md).
