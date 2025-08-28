# CARLA Project - Hero Vehicle with NPCs and Avoidance

This project is based on [CARLA Simulator 0.9.15](https://carla.org/).  
It demonstrates:
- Spawning a **Hero vehicle**
- Spawning multiple **NPC vehicles** controlled by Traffic Manager
- Hero vehicle has a **basic avoidance logic** (lane following, slowing down/braking for leading vehicles, lateral avoidance of nearby vehicles)
- **Spectator camera** automatically follows the Hero vehicle (third-person smooth view)
- Vehicles are **not destroyed** when the script ends (so you can accumulate actors if needed)

---

## Requirements

- **CARLA 0.9.15** installed and running
- **Python 3.7.x (64-bit)**  
  (CARLA official wheels only support Python 3.7)
  ```bash
  winget install -e --id Python.Python.3.7

- **Install from the local wheel:**:
  ```bash
  pip install /path/to/carla-0.9.15-cp37-cp37m-win_amd64.whl


## Start Carla server
From a terminal (adjust path to your installation and parameter by your device performance):
```bash
"D:\carla_0.9.15\CarlaUE4.exe" -quality-level=Low -ResX=640 -ResY=360 -windowed -nosound -fps=20
```

In another terminal:
```bash
python car.py
```
