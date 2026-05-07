import pickle
from pathlib import Path
import numpy as np

#.pkl gibt die Daten in folgendem Format aus:
#1. joint_positions in rad
#2. joint_velocities in rad/s
#3. end-effector pose, evtl nicht korrekt
#4. gripper zustand (0=geschlossen, 1=offen). Achtung: Die Werte weichen teilweise ab 
#5. controlm = Steuerbefehl an realen Emika Franka Panda

pkl_path = Path("/home/tim/bc_data/gello/0507_104424/2026-05-07T10:44:24.687412.pkl")

with open(pkl_path, "rb") as f:
    data = pickle.load(f)

print("Datei:", pkl_path)
print("Typ:", type(data))

if isinstance(data, dict):
    print("\nKeys und Inhalte:")
    for key, value in data.items():
        print("\n---", key, "---")
        print("type:", type(value))
        print("shape:", getattr(value, "shape", None))

        if isinstance(value, np.ndarray):
            if value.size <= 30:
                print(value)
            else:
                print(value.flatten()[:30])
        else:
            print(value)
else:
    print(data)

