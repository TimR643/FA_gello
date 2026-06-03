# CRISP-Übergang mit bestehenden GELLO/LeRobot-Daten

Diese Notiz bereitet den Test vor, ob deine vorhandenen GELLO-Demonstrationen als
LeRobot-Dataset grundsätzlich CRISP-fähig sind. Ziel ist, Datenfehler von Fehlern
im bisherigen Polymetis/ZMQ-Rollout zu trennen.

## Kurze Antwort

Ja, du kannst deine gespeicherten Daten **direkt als Ausgangspunkt für CRISP**
verwenden, sofern sie ein gültiges `LeRobotDataset` sind und die CRISP-Umgebung
dieselben Keys, Dimensionen, Kamera-Namen, Gripper-Konventionen und Policy-FPS
nutzt.

CRISP selbst stellt laut Dokumentation zwei dafür relevante Bausteine bereit:

1. `crisp_gym` kann Trajektorien im `LeRobotDataset`-Format aufzeichnen.
2. `crisp_gym` kann trainierte LeRobot-Policies deployen.

Der wichtigste Punkt: CRISP löst nicht automatisch Dataset-Schema-Probleme. Wenn
deine Daten `observation.images.wrist` enthalten, dein SmolVLA-Training aber auf
`observation.images.camera1` umbenannt wurde, muss diese Zuordnung auch im
CRISP-Deployment wieder identisch hergestellt werden.

## Was mit diesem Branch vorbereitet ist

Dieser Branch enthält ein lokales Prüfskript:

```bash
python scripts/validate_lerobot_crisp_ready.py \
  --dataset-root /home/tim/lerobot_data/left_right/left_right_test \
  --expected-camera wrist \
  --expected-fps 10
```

Das Skript prüft ohne CRISP-Installation:

- `meta/info.json` existiert.
- `observation.state` hat die erwartete Panda/GELLO-Shape `(8,)`.
- `action` hat die erwartete Panda/GELLO-Shape `(8,)`.
- `observation.images.<camera>` ist im Dataset deklariert.
- Metadaten-FPS passt zur erwarteten Policy-Frequenz.
- Falls ein Parquet-Reader verfügbar ist (`pyarrow` bevorzugt, sonst
  `pandas.read_parquet`): Parquet-Frames werden stichprobenartig geprüft,
  inklusive State-/Action-Längen, NaN/Inf, Timestamp-FPS, Dimensionsbereichen
  und Gripper-State/Action-Korrelation.

## Exakter Testplan

### 1. Dataset lokal prüfen

Führe zuerst den Readiness-Check aus:

```bash
python scripts/validate_lerobot_crisp_ready.py \
  --dataset-root "$DATASET_ROOT" \
  --expected-camera wrist \
  --expected-fps 10 \
  --max-parquet-files 5 \
  --require-frame-inspection
```

Falls du nur die Meldung `No parquet reader is installed` oder `pyarrow is not
installed` siehst, hast du bisher nur die Metadaten geprüft. Installiere dann im
gleichen Python-/Conda-Environment einen Parquet-Reader und wiederhole den
Check:

```bash
python3 -m pip install pyarrow
```

Bewertung:

- `FAIL` bedeutet: Datensatz vor CRISP korrigieren.
- `WARN` bedeutet: nicht zwingend kaputt, aber für den Hardware-Test kritisch
  prüfen.
- Eine besonders wichtige Warnung/Fehlermeldung ist negative
  Gripper-Korrelation. Das deutet auf `open=1` vs. `open=0`-Vertauschung hin.

### 2. CRISP-/ROS2-Umgebung aufsetzen

In einer separaten CRISP-Arbeitskopie:

```bash
git clone https://github.com/utiasDSL/crisp_gym
cd crisp_gym
mkdir -p scripts
cat > scripts/set_env.sh <<'SH'
export GIT_LFS_SKIP_SMUDGE=1
export SVT_LOG=1
export ROS_DOMAIN_ID=100
export CRISP_CONFIG_PATH=/path/to/your/crisp/configs
SH
GIT_LFS_SKIP_SMUDGE=1 pixi install -e humble-lerobot
pixi shell -e humble-lerobot
python -c "import crisp_gym"
pixi run -e humble-lerobot crisp-check-config
```

Wenn du Franka/Panda über den CRISP-Controller fahren willst, brauchst du
zusätzlich eine lauffähige ROS2-/Franka-Controller-Konfiguration für deinen
Roboter. Für Panda/FR3 verweist CRISP auf bestehende Franka-Beispiele und eigene
Controller-Configs.

### 3. Daten gegen CRISP-Schema spiegeln

Lege in CRISP eine `ManipulatorEnvConfig` so an, dass sie dieselben Signale wie
dein Dataset liefert:

| Dein Dataset | CRISP/Policy muss sehen |
| --- | --- |
| `observation.state`, Shape `(8,)` | identische Gelenkreihenfolge |
| `action`, Shape `(8,)` | identische Ziel-Konvention |
| `observation.images.wrist` | identischer Key oder identischer Rename |
| `fps=10` | gleiche effektive Deploy-Rate |

Für SmolVLA mit deinem bisherigen Rename gilt zusätzlich:

```text
observation.images.wrist -> observation.images.camera1
observation.images.camera2 -> dummy/empty camera
observation.images.camera3 -> dummy/empty camera
```

Das muss im CRISP-Deployment genauso passieren wie in deinem aktuellen
`experiments/run_lerobot_real_robot.py`.

### 4. Policy in CRISP deployen

In der CRISP-Umgebung:

```bash
pixi run -e humble-lerobot crisp-deploy-policy --path /path/to/pretrained_model
```

Starte zuerst ohne Objektkontakt und mit sehr konservativen Controller-Settings.
Prüfe dabei:

1. State-Key und Bild-Key werden von der Policy akzeptiert.
2. Die Policy-Action hat Shape `(8,)`.
3. Die ersten 7 Gelenke bleiben im erwarteten Panda-Gelenkraum.
4. Der Gripper bewegt in der richtigen Richtung.
5. CRISP-Controller läuft mit der geplanten Policy-Frequenz.

## Entscheidungsmatrix

| Ergebnis | Interpretation |
| --- | --- |
| Dataset-Check schlägt fehl | Fehler liegt wahrscheinlich in Daten/Semantik. |
| Dataset-Check ok, CRISP-Dry-Run/Deploy akzeptiert Keys nicht | Fehler liegt im CRISP-Config-/Rename-Schema. |
| CRISP bewegt sinnvoll, dein ZMQ-Rollout nicht | Fehler liegt wahrscheinlich in deinem bisherigen Rollout/Pre-/Postprocessing/Polymetis-Bridge. |
| CRISP und ZMQ zeigen denselben Fehler | Fehler liegt wahrscheinlich in Daten, Training oder Action-Konvention. |

## Bekannte Risiken aus deinem aktuellen Stack

1. **Gripper-Konvention:** GELLO-Gripper und Panda-State können invertiert sein.
   Das Prüfskript meldet eine stark negative State/Action-Korrelation als
   Fehler.
2. **FPS-Mismatch:** Deine `run_env.py`-Control-Loop kann mit 100 Hz laufen,
   während das Dataset als 10 FPS deklariert ist. Das Skript vergleicht
   Metadaten-FPS und, mit Parquet-Reader, Timestamp-FPS.
3. **Kamera-Rename:** SmolVLA erwartet bei dir ggf. `camera1`, während dein
   Dataset ursprünglich `wrist` enthält.
4. **Action-Modus:** Deine aktuellen GELLO-Actions sind absolute Joint-Ziele.
   Wenn CRISP Deltas erwartet, muss ein Adapter dazwischen.
