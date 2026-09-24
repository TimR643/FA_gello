# Minimaler Skriptbestand auf dem Franka-Laptop

Diese Liste gilt fuer den aktuellen Zwei-Kamera-Ablauf: Polymetis wird zuerst
gestartet, danach laeuft `start_gello_panda.sh`, und der Laptop stellt seine
lokalen ZMQ-Ports ueber einen Reverse-SSH-Tunnel auf dem HPC bereit.

## Fuer jeden Start erforderlich

Auf dem Franka-Laptop muessen diese Einstiegsskripte vorhanden sein:

| Datei | Aufgabe |
| --- | --- |
| `start_gello_panda.sh` | Erstellt die tmux-Session und fuellt die Befehle fuer Robotik, Greifer, beide Kameras und Teleoperation vor. Die Befehle werden danach in der dokumentierten Reihenfolge manuell mit Enter gestartet. |
| `start_franka_to_hpc_reverse_tunnel.sh` | Leitet Robot (`6001`), Wrist-Kamera (`5000`) und Base-Kamera (`5001`) per Reverse SSH zum HPC weiter. |

`start_gello_panda.sh` ruft ausserdem genau diese drei Python-Skripte auf. Sie
duerfen deshalb ebenfalls nicht entfernt werden:

* `experiments/launch_nodes.py`
* `experiments/launch_camera_single.py`
* `experiments/run_env.py`

Die Befehle fuer `launch_robot.py` und `launch_gripper.py` kommen aus der
installierten Polymetis-Umgebung und sind keine Dateien dieses Repositories.

## Sinnvoll auf dem Laptop, aber nicht startkritisch

* `stop_gello_panda.sh` beendet die tmux-Session und verbliebene Prozesse
  kontrolliert. Fuer das Hochfahren ist es nicht erforderlich.
* `calibrate_gello.sh` und `scripts/gello_get_offset.py` werden nur fuer eine
  erneute GELLO-Kalibrierung benoetigt.
* `check_gello_start_position.sh`, `move_gello_start_position.sh` sowie die
  gleichnamigen Python-Skripte unter `scripts/` sind Hilfen zum Pruefen oder
  Anfahren einer definierten Startpose. Der oben beschriebene Start ruft sie
  nicht auf.
* `view_wrist_camera.sh`, `view_base_camera.sh` und
  `scripts/view_zmq_camera.py` sind reine Diagnosewerkzeuge.

## Nur auf dem HPC erforderlich

`start_hpc_remote_camera_recorder.sh` und
`experiments/record_lerobot_stream_with_remote_cameras.py` gehoeren zur
Aufnahmeseite auf dem HPC. Die `start_*policy*hpc.sh`- und nativen
Policy-/Inference-Launcher werden ebenfalls auf dem HPC ausgefuehrt. Sie koennen
aus einer **separaten Laptop-Kopie** entfernt werden, muessen aber in der
HPC-Kopie erhalten bleiben, falls sie dort weiterhin verwendet werden.

## Was nicht entfernt werden darf

Die drei genannten Python-Einstiegsskripte importieren Code aus `gello/`.
Darum ist nicht nur die sichtbare Liste der Startskripte relevant: Das
Python-Paket `gello/`, `setup.py` beziehungsweise die installierte editable
Installation und die benoetigten Abhaengigkeiten muessen auf dem Laptop
erhalten bleiben. Auch `start_gello_panda.sh` darf nicht ohne Anpassung von
einer bestehenden Installation getrennt werden.

Vor dem Loeschen kann der tatsaechliche Ablauf ohne Roboterbewegung zumindest
statisch geprueft werden:

```bash
bash -n start_gello_panda.sh start_franka_to_hpc_reverse_tunnel.sh stop_gello_panda.sh
python -m py_compile \
  experiments/launch_nodes.py \
  experiments/launch_camera_single.py \
  experiments/run_env.py
```

Nach dem Start muessen lokal alle drei weitergeleiteten Ports lauschen:

```bash
ss -ltnp | grep -E ':(6001|5000|5001)'
```

Erst danach wird `start_franka_to_hpc_reverse_tunnel.sh` gestartet. Der Tunnel
muss waehrend Aufnahme oder Inferenz geoeffnet bleiben.
