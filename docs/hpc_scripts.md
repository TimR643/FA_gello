# Aktive Skripte auf dem HPC

Diese Liste dokumentiert die bewusst beibehaltenen HPC-Einstiegspunkte und ihre
internen Abhängigkeiten. Alte, durch diese Einstiegspunkte ersetzte Launcher
wurden entfernt.

## Aufnahme und Diagnose

- `start_hpc_remote_camera_recorder.sh` zeichnet den vom Roboter-Laptop
  gesendeten Zustands-/Aktionsstrom zusammen mit den entfernten Kamerabildern
  auf. Das Skript startet intern
  `experiments/record_lerobot_stream_with_remote_cameras.py`.
- `view_wrist_camera.sh` und `view_base_camera.sh` zeigen die jeweilige Kamera
  über den SSH-/ZMQ-Tunnel. Beide benötigen intern
  `scripts/view_zmq_camera.py`.
- `check_gello_start_position.sh --watch` beobachtet fortlaufend die
  Gelenkpositionen. Dafür muss `scripts/check_gello_start_position.py`
  beibehalten werden.

## Inference

Die aktiven Launcher sind:

- `start_smolvla_native_inference.sh`
- `start_pi05_multi_episode.sh`
- `start_lerobot_smolvla_multi_episode.sh`
- `start_lerobot_native_act_policy.sh`
- `start_lerobot_native_act_policy_multi_episode.sh`
- `start_pi0.5_inference_fast.sh`

Zusätzlich bleibt `start_lerobot_native_real_policy.sh` erhalten: Die beiden
ACT-Launcher rufen dieses generische Skript intern auf. Es ist daher keine
ungenutzte Alternative, auch wenn es nicht direkt gestartet wird.

## Roboter-Laptop

Die Launcher für GELLO, Kameraserver, Reverse-Tunnel, Kalibrierung und Stoppen
der Prozesse bleiben ebenfalls erhalten. Sie laufen zwar nicht auf dem HPC,
gehören aber zum Laptop-Teil desselben Aufnahme- und Inference-Aufbaus.
