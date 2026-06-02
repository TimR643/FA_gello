# LeRobot-naher Real-Robot-Rollout

Dieser Branch bereitet den sichereren Rückweg vom trainierten LeRobot-Modell auf den echten GELLO/ZMQ-Roboter vor.

## Ziel

Der Live-Test soll möglichst nah am LeRobot-Inference-Vertrag bleiben:

1. Checkpoint und Dataset-Metadaten laden.
2. Live-Observation aus `RobotEnv` lesen.
3. Live-Daten in die gleichen LeRobot-Keys umwandeln, die beim Training verwendet wurden:
   - `observation.state`
   - `observation.images.wrist`
   - optional `observation.images.base`
4. `policy.select_action(batch)` ausführen.
5. Policy-Action validieren und sicher in ein Robot-Kommando umwandeln.

## Neuer Einstiegspunkt

```bash
python experiments/run_lerobot_real_robot.py \
  --checkpoint /path/to/checkpoints/010000/pretrained_model \
  --dataset-root /path/to/lerobot_dataset \
  --repo-id local/panda_gello \
  --cameras wrist \
  --duration 2.0 \
  --hz 5.0
```

Ohne `--execute` ist das ein Dry-Run: Das Script liest Live-Observationen und fragt die Policy ab, bewegt den Roboter aber nicht.

## Erster echter Test

```bash
python experiments/run_lerobot_real_robot.py \
  --checkpoint /path/to/checkpoints/010000/pretrained_model \
  --dataset-root /path/to/lerobot_dataset \
  --repo-id local/panda_gello \
  --cameras wrist \
  --duration 1.0 \
  --hz 2.0 \
  --max-joint-delta 0.005 \
  --max-gripper-delta 0.01 \
  --execute
```

Für die ersten Tests sollte der Arbeitsraum leer sein und die Hand am Enable-/Not-Aus-Schalter bleiben.

## Sicherheitsannahmen

Die Standardeinstellung `--action-mode absolute_joint_position` passt zu den aktuellen GELLO-Daten: Die aufgezeichnete `action` entspricht einem absoluten Gelenkziel. Wenn künftig Deltas trainiert werden, muss `--action-mode delta_joint_position` verwendet werden.

Das Script bricht ab, wenn:

- Live-State nicht Shape `(8,)` hat,
- Bilder nicht RGB `480x640x3` sind,
- erwartete Kamera-Keys fehlen,
- Policy-Output nicht Shape `(8,)` hat,
- Policy-Output `NaN` oder `Inf` enthält,
- Live-Batch-Keys nicht in den LeRobot-Dataset-Metadaten auftauchen.

## Wo die LeRobot-Anpassung sitzt

Die wiederverwendbaren Bausteine liegen in `gello/lerobot/real_robot.py`:

- `load_lerobot_policy(...)` lädt Checkpoint, Dataset-Meta und Policy über LeRobot.
- `LeRobotObservationAdapter` baut aus `RobotEnv`-Observations den LeRobot-Policy-Batch.
- `SafeJointActionExecutor` begrenzt Policy-Actions vor `env.step(...)`.

Damit bleibt nur die Hardware-Anbindung GELLO/ZMQ-spezifisch; Policy-Laden und Batch-Schema bleiben nah am LeRobot-Workflow.

## SmolVLA / HPC-Konfiguration

Für SmolVLA-Policies, die mit `--rename_map='{"observation.images.wrist": "observation.images.camera1"}'` und `--policy.empty_cameras=2` trainiert wurden, darf die ursprüngliche Dataset-Meta beim Erzeugen der Policy nicht erneut in `make_policy` hineingereicht werden. Sonst kollidieren die Dataset-Keys `observation.images.wrist` mit den Checkpoint-Keys `observation.images.camera1`, `observation.images.camera2` und `observation.images.camera3`.

Die SmolVLA-Einstiegspunkte setzen deshalb standardmäßig `use_dataset_meta=False`. Runtime-Mapping:

- `observation.images.wrist` wird zu `observation.images.camera1` umbenannt.
- `observation.images.camera2` und `observation.images.camera3` werden als Nullbilder mit gleicher Form wie `camera1` ergänzt.
- ACT-Rollouts verwenden weiter die Dataset-Meta und laufen über `experiments/run_lerobot_real_robot_act.py`.

Auf dem HPC können die Launcher-Pfade ohne Editieren der Datei überschrieben werden:

```bash
CKPT=/pfad/zum/pretrained_model \
DATASET_ROOT=/pfad/zum/lerobot_dataset \
GELLO_ROOT=/pfad/zum/gello_software \
./start_smolvla_policy_hpc.sh
```

Für einen nicht-interaktiven Dry-Run:

```bash
EXECUTE=0 REQUIRE_ENTER=0 ./start_smolvla_policy_hpc.sh
```
