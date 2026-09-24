# HPC-Software für GELLO, Franka Panda und LeRobot

Dieser Branch enthält die Softwarekomponenten, die auf dem **HPC** für die
Aufzeichnung von GELLO-Demonstrationen und für die autonome Inference auf einer
Franka Emika Panda ausgeführt werden. Die Hardware-, Polymetis-, GELLO- und
Kameraserver laufen weiterhin auf dem **Franka-Laptop**. Beide Rechner tauschen
Roboterzustände, Aktionen und Kamerabilder über ZMQ und gegebenenfalls einen
SSH-Tunnel aus.

> **Sicherheit:** Vor jeder Bewegung müssen Arbeitsraum, Not-Aus und
> Kollisionsfreiheit geprüft werden. Während einer autonomen Episode muss eine
> Person am Roboter bleiben und die Episode jederzeit abbrechen können.

## 1. Verantwortlichkeiten der beiden Rechner

### Franka-Laptop

Der Franka-Laptop ist für die hardwarenahe Seite zuständig:

- Polymetis-Roboter- und Gripperserver,
- GELLO-Steuerung bei Demonstrationsaufnahmen,
- ZMQ-Roboterserver auf Port `6001`,
- Wrist-Kamera auf Port `5000`,
- Base-Kamera auf Port `5001`,
- Anfahren der zur Aufgabe passenden Ausgangsposition,
- optional: Aufbau des Reverse-SSH-Tunnels zum HPC.

### HPC

Der HPC übernimmt:

- das Zusammenführen und Speichern der Demonstrationsdaten,
- den Zugriff auf Roboter und Kameras über ZMQ/SSH,
- das Laden der trainierten Checkpoints,
- die autonome Policy-Inference,
- optionales LeRobot- und H5-Logging während der Inference.

## 2. Voraussetzungen

Auf dem HPC werden eine funktionierende LeRobot-Umgebung und dieses Repository
erwartet. Die Launcher verwenden standardmäßig folgende Pfade:

```text
Repository:   ~/gello_software
Conda-Setup:  ~/miniconda3/etc/profile.d/conda.sh
Conda-Env:    lerobot beziehungsweise lerobot_current
```

Abweichende Pfade können bei den meisten Skripten über `GELLO_REPO_DIR`,
`PROJECT_DIR`, `CONDA_SH`, `CONDA_SETUP` und `LEROBOT_ENV` gesetzt werden.

Vor Aufnahme oder Inference müssen vom HPC aus erreichbar sein:

| Dienst | Standardport |
|---|---:|
| Wrist-Kamera | `5000` |
| Base-Kamera | `5001` |
| Roboter-ZMQ | `6001` |
| Aufnahmestrom zum HPC | `7000` |

Bei Verwendung des Reverse-Tunnels wird auf dem Franka-Laptop nach dem Start
der lokalen Server ausgeführt:

```bash
./start_franka_to_hpc_reverse_tunnel.sh
```

Danach sind Roboter und Kameras für die HPC-Skripte üblicherweise unter
`127.0.0.1` erreichbar. Bei direkter Netzwerkverbindung muss stattdessen die IP
des Franka-Laptops als `ROBOT_HOST` beziehungsweise `HPC_CAMERA_HOST` verwendet
werden.

## 3. Demonstrationen aufzeichnen

### 3.1 Datensatzparameter festlegen

**Vor dem Start einer Aufnahmeepisode muss der HPC-Recorder erfolgreich
laufen.** Für jede Aufgabe müssen mindestens ein eindeutiger Ausgabeordner, eine
Repo-ID und der richtige Aufgabenprompt festgelegt werden:

```bash
export LEROBOT_ROOT="$HOME/lerobot_data/bottle_task"
export LEROBOT_REPO_ID="local/bottle_task"
export LEROBOT_TASK="Lift the bottle and put it to the right-hand side space"
```

`LEROBOT_ROOT` darf nicht versehentlich auf einen Datensatz einer anderen
Aufgabe zeigen. `LEROBOT_REPO_ID` und `LEROBOT_TASK` müssen zum Inhalt der
aufgezeichneten Episoden passen. Die Werte können im Launcher geändert werden;
empfohlen wird jedoch die Übergabe als Umgebungsvariablen, damit das Skript für
verschiedene Aufgaben unverändert bleibt.

Bei einem Reverse-SSH-Tunnel ist der Kamerahost auf dem HPC normalerweise
`127.0.0.1`. Bei direktem Netzwerkzugriff wird die Laptop-IP eingetragen:

```bash
export HPC_CAMERA_HOST="127.0.0.1"       # Reverse-Tunnel
# export HPC_CAMERA_HOST="192.168.1.50"  # direkter Zugriff auf den Laptop
```

### 3.2 HPC-Recorder zuerst starten

Auf dem HPC:

```bash
cd ~/gello_software
./start_hpc_remote_camera_recorder.sh
```

Vor dem Fortfahren muss die ausgegebene Konfiguration kontrolliert werden:

- `Laptop camera host`
- `Dataset root`
- `Repo id`
- `FPS`
- `Task`

Der Recorder lauscht auf Port `7000`, empfängt Zustände und Aktionen vom
Franka-Laptop und holt die Bilder separat von Wrist- und Base-Kamera. Intern
startet er `experiments/record_lerobot_stream_with_remote_cameras.py`.

### 3.3 Franka-Laptop starten

Erst wenn der HPC-Recorder bereit ist, wird auf dem Franka-Laptop der
Aufnahmeaufbau gestartet. Dabei muss `HPC_RECORD_HOST` auf die vom Laptop
erreichbare HPC-Adresse zeigen:

```bash
export HPC_RECORD_HOST="<HPC-IP-ODER-HOSTNAME>"
cd ~/gello_software
./start_gello_panda.sh
```

Das Skript startet Roboter, Gripper, ZMQ-Knoten, beide Kameraserver und zuletzt
die GELLO-Umgebung mit dem Zustands-/Aktionsstrom zum HPC. Anschließend kann die
Demonstration mit GELLO ausgeführt und über die Aufnahmeoberfläche beendet oder
bestätigt werden.

### 3.4 Empfohlene Reihenfolge pro Aufnahmesitzung

1. Aufgabe, Szene und gewünschtes Datensatzverzeichnis festlegen.
2. Auf dem HPC `LEROBOT_ROOT`, `LEROBOT_REPO_ID`, `LEROBOT_TASK` und den
   Kamerahost prüfen.
3. `start_hpc_remote_camera_recorder.sh` auf dem HPC starten.
4. Kontrollieren, dass der Recorder ohne Fehler läuft.
5. Roboter- und Kameraserver auf dem Franka-Laptop starten.
6. Falls benötigt, SSH-Tunnel herstellen und die Erreichbarkeit prüfen.
7. Roboter und Szene in die definierte Ausgangslage bringen.
8. Erst danach die eigentliche Demonstration starten.

## 4. Kameras und Gelenkpositionen prüfen

### Kamerabilder

Auf dem HPC können die über ZMQ/SSH übertragenen Kamerabilder einzeln geprüft
werden:

```bash
./view_wrist_camera.sh
./view_base_camera.sh
```

Bei direkter Verbindung kann der Host vorangestellt werden:

```bash
CAMERA_HOST="<FRANKA-LAPTOP-IP>" ./view_wrist_camera.sh
```

### Gelenkpositionen beobachten

Die aktuelle Position kann einmalig oder kontinuierlich geprüft werden:

```bash
./check_gello_start_position.sh
./check_gello_start_position.sh --watch --period-s 5
```

`--watch` wird mit `Ctrl+C` beendet.

> **Wichtig:** Kamera-Preview und `--watch` vor Aufnahme oder Inference wieder
> schließen. Zusätzliche ZMQ-Abfragen können sonst Kamera-FPS und Timing des
> Kontrollloops beeinträchtigen.

## 5. Ausgangsposition vor der Inference

Vor **jeder** autonomen Episode muss die für Aufgabe und Checkpoint verwendete
Ausgangsposition auf dem Franka-Laptop angefahren werden. Die Position muss der
Startverteilung der Trainingsdaten entsprechen; ein falscher Startzustand kann
zu unvorhersehbaren Policy-Aktionen führen.

Auf dem Franka-Laptop wird dafür das zur Aufgabe gehörende Startpositionsskript
verwendet. Der vorhandene Python-Helfer kann beispielsweise so aufgerufen
werden, nachdem Roboter-ZMQ erreichbar ist:

```bash
cd ~/gello_software
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot

python scripts/move_gello_start_position.py \
  --robot-host 127.0.0.1 \
  --robot-port 6001 \
  --target-rad "<q1,q2,q3,q4,q5,q6,q7>" \
  --target-gripper 1.0
```

Die sieben Zielwinkel müssen für die konkrete Aufgabe dokumentiert und bewusst
eingetragen werden. Die Position darf nicht aus einem anderen Datensatz kopiert
werden, ohne sie am Roboter zu validieren. Danach sollte die Position über den
Checker verifiziert werden. Während die Position automatisch angefahren wird,
muss der Arbeitsraum frei sein.

## 6. Checkpoint und Aufgabenprompt für Inference konfigurieren

Jeder Inference-Launcher muss den **zur Aufgabe passenden Checkpoint** und den
gleichen semantischen Aufgabenprompt wie beim Training erhalten. Am sichersten
werden beide Werte beim Start explizit gesetzt:

```bash
CKPT="/pfad/zu/checkpoints/060000/pretrained_model" \
TASK="Put all red objects into the red box and all other objects into the white box." \
./start_smolvla_native_inference.sh
```

Ein lokaler Checkpoint muss normalerweise ein `pretrained_model`-Verzeichnis
mit mindestens einer `config.json` sein. Vor dem Start prüfen:

```bash
test -f "$CKPT/config.json"
```

Pi0.5-Launcher können alternativ mit `HF_MODEL_REPO` und `HF_CHECKPOINT` einen
Checkpoint vom Hugging-Face Hub auswählen. Ein explizites `CKPT` hat dabei
Vorrang, sofern das jeweilige Skript dies unterstützt.

## 7. Verfügbare Inference-Launcher

### SmolVLA, einzelne Episode

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
./start_smolvla_native_inference.sh
```

Dieser Launcher ist speziell für SmolVLA, verwendet standardmäßig RTC-Inference
und filtert virtuelle beziehungsweise nicht vorhandene Kameraschlüssel.

### SmolVLA, mehrere Episoden

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
NUM_EPISODES=20 \
EPISODE_TIME_S=35 \
RESET_TIME_S=15 \
./start_lerobot_smolvla_multi_episode.sh
```

Während `RESET_TIME_S` wird die Szene zurückgesetzt und auf dem Franka-Laptop
erneut die Ausgangsposition angefahren. Erst danach darf die nächste Episode
beginnen.

### Pi0.5, schnelle einzelne Inference

Mit lokalem Checkpoint:

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
./start_pi0.5_inference_fast.sh
```

Oder mit Hub-Checkpoint:

```bash
HF_MODEL_REPO="organisation/modell" \
HF_CHECKPOINT="010000" \
TASK="<Aufgabenprompt>" \
./start_pi0.5_inference_fast.sh
```

### Pi0.5, mehrere Episoden

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
NUM_EPISODES=20 \
EPISODE_TIME_S=35 \
RESET_TIME_S=12 \
./start_pi05_multi_episode.sh
```

Das Modell bleibt dabei über mehrere Episoden geladen. Die Reset-Pause ist für
das Zurücksetzen der Szene und das erneute Anfahren der Ausgangsposition
vorgesehen.

### ACT, einzelne Episode

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
./start_lerobot_native_act_policy.sh
```

### ACT, mehrere Episoden

```bash
CKPT="/pfad/zum/pretrained_model" \
TASK="<Aufgabenprompt>" \
NUM_EPISODES=20 \
EPISODE_TIME_S=35 \
RESET_TIME_S=15 \
./start_lerobot_native_act_policy_multi_episode.sh
```

Die beiden ACT-Launcher verwenden intern
`start_lerobot_native_real_policy.sh`. Dieses generische Skript darf deshalb
nicht gelöscht werden, auch wenn es normalerweise nicht direkt gestartet wird.

## 8. Verbindliche Reihenfolge für autonome Inference

1. Richtige Aufgabe und richtigen Checkpoint auswählen.
2. Roboter, Gripper und Kameraserver auf dem Franka-Laptop starten.
3. Reverse-SSH-Tunnel starten oder direkte Erreichbarkeit konfigurieren.
4. Wrist- und Base-Kamera kurz vom HPC aus prüfen und die Preview schließen.
5. Auf dem Franka-Laptop die aufgabenspezifische Ausgangsposition anfahren.
6. Gelenkposition kontrollieren und `--watch` anschließend schließen.
7. Szene entsprechend der Trainingsdaten aufbauen.
8. `CKPT` und `TASK` im gewählten Inference-Launcher kontrollieren.
9. Sicherstellen, dass keine Person und kein Gegenstand unerwartet im
   Arbeitsraum ist.
10. Inference starten und den Roboter durchgehend überwachen.
11. Bei Multi-Episode-Läufen jede Reset-Pause vollständig für Szenenreset und
    Ausgangsposition nutzen.

## 9. Optionales Logging während der Inference

Die Launcher unterstützen je nach Skript zusätzliche Optionen:

```bash
./start_smolvla_native_inference.sh --h5-log
./start_smolvla_native_inference.sh --record-lerobot
```

`--h5-log` erzeugt Inference-/Joint-Logs. `--record-lerobot` legt zusätzlich
einen LeRobot-Rollout-Datensatz an und kann die Inference verlangsamen. Vor einer
produktiven Aufnahme sollten Pfade, freier Speicherplatz und FPS mit einer
kurzen Testepisode kontrolliert werden.

## 10. Fehlerdiagnose

### Kamera oder Roboter nicht erreichbar

Auf dem HPC prüfen:

```bash
timeout 2 bash -c '</dev/tcp/127.0.0.1/6001'
timeout 2 bash -c '</dev/tcp/127.0.0.1/5000'
timeout 2 bash -c '</dev/tcp/127.0.0.1/5001'
```

Bei einem Fehler kontrollieren:

- laufen die ZMQ-Server auf dem Franka-Laptop,
- läuft der Reverse-Tunnel noch,
- stimmen Hostnamen und Ports,
- blockiert eine Firewall die direkte Verbindung,
- läuft noch ein Kamera-Preview-Prozess.

### Checkpoint wird nicht akzeptiert

```bash
test -d "$CKPT"
test -f "$CKPT/config.json"
```

Zusätzlich muss der Policy-Typ zum Launcher passen: SmolVLA-Checkpoints werden
mit einem SmolVLA-Launcher und ACT-Checkpoints mit einem ACT-Launcher gestartet.

### Falscher oder bestehender Aufnahmeordner

Vor jeder neuen Aufgabe `LEROBOT_ROOT`, `LEROBOT_REPO_ID` und `LEROBOT_TASK`
erneut kontrollieren. Einen bestehenden Datensatz nicht löschen oder
überschreiben, bevor geprüft wurde, ob der Recorder Episoden sicher anhängen
kann.

## 11. Stoppen

Inference- und Preview-Prozesse werden im jeweiligen Terminal mit `Ctrl+C`
beendet. Die Laptop-seitigen GELLO-/Polymetis-Prozesse können anschließend auf
dem Franka-Laptop mit folgendem Skript gestoppt werden:

```bash
./stop_gello_panda.sh
```

## Lizenz und Herkunft

Dieses Repository basiert auf der ursprünglichen
[GELLO-Software von Philipp Wu](https://github.com/wuphilipp/gello_software).
Copyright- und Lizenzhinweise befinden sich in `LICENSE`.
