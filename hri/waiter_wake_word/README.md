# Wake Word Perception Node (ROS 2 Humble)

**Package:** `waiter_wake_word`  
**Workspace:** `robot_waiter_ws`  
**Role:** Voice wake-word perception for the Robot Waiter project

This package provides a **standalone wake‑word detection node** for ROS 2 Humble using **OpenWakeWord** and a microphone input. It publishes clean ROS topics that downstream systems (FSM, HRI, navigation, LEDs) can subscribe to.

✅ **Key goal:** Make it reproducible on **any** Ubuntu 22.04 + ROS 2 Humble machine (laptop, Jetson, TurtleBot4), without breaking other ROS Python stacks.

---

## Features

- Real‑time wake‑word detection (tested with `hey_jarvis`)
- Debounce & cooldown logic (prevents repeated triggers)
- Clean ROS topic interface
- Parameterised via ROS params (model, threshold, mic, timings)
- Works on laptop **and** Jetson

---

## Topics

### Published Topics

| Topic | Type | Description |
|------|------|-------------|
| `/wake_word/activated` | `std_msgs/Bool` | Pulses `True` when wake word detected |
| `/robot/state` | `std_msgs/String` | `listening` → `idle` |
| `/led/command` | `std_msgs/String` | `listening_mode` → `idle_mode` |
| `/wake_word/score` | `std_msgs/Float32` | Smoothed confidence score (EMA) |

> **Design note:** No subscribers are required. This node is purely perceptual and stateless beyond cooldown handling.

---

## Parameters

| Parameter | Default | Description |
|---------|---------|-------------|
| `wake_word_model` | `hey_jarvis` | Wake‑word model name |
| `threshold` | `0.55` | Detection confidence threshold |
| `sample_rate` | `16000` | Microphone sample rate |
| `channels` | `1` | Audio channels |
| `chunk_size` | `1280` | Audio frame size (80 ms) |
| `mic_device_index` | `-1` | `-1` = default microphone |
| `cooldown_s` | `2.0` | Suppression window after detection |
| `listening_hold_s` | `5.0` | Time to stay in listening state |
| `score_smoothing_alpha` | `0.35` | EMA smoothing factor |
| `log_period_s` | `10.0` | Stats log interval |

---

## Directory Structure

```text
waiter_wake_word/
├── launch/
│   └── wake_word.launch.py
├── resource/
├── waiter_wake_word/
│   ├── __init__.py
│   └── wake_word_node.py
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

Models are **not stored inside the package** and must be provided externally.

---

## Python Environment (Recommended + Reproducible)

### Why a venv?
OpenWakeWord pulls in scientific Python deps (NumPy/SciPy/sklearn/onnxruntime). ROS installs its own Python packages too. Mixing them can cause crashes like:

- `A module that was compiled using NumPy 1.x cannot be run in NumPy 2.x`
- `AttributeError: _ARRAY_API not found`
- `ImportError: numpy.core.multiarray failed to import`

To avoid this, use an isolated venv for wake-word.

### Create venv (once)

```bash
cd ~/robot_waiter_ws
python3 -m venv .venv_wake
source .venv_wake/bin/activate

python3 -m pip install --upgrade pip setuptools wheel
python3 -m pip install \
  openwakeword==0.6.0 \
  pyaudio==0.2.14 \
  numpy==1.26.4 \
  scipy==1.11.4 \
  scikit-learn==1.3.2
```

If `pyaudio` fails:

```bash
sudo apt update
sudo apt install -y portaudio19-dev
python3 -m pip install pyaudio==0.2.14
```

✅ Verify (should print venv paths):

```bash
which python3
python3 -c "import numpy; print(numpy.__version__, numpy.__file__)"
python3 -c "import openwakeword; print(openwakeword.__file__)"
python3 -c "import pyaudio; print('pyaudio OK')"
```

---

## Download Wake‑Word Models

We keep models in the workspace so paths are explicit and reproducible.

```bash
source ~/robot_waiter_ws/.venv_wake/bin/activate
mkdir -p ~/robot_waiter_ws/wakeword_models

python3 - <<'PY'
from openwakeword.utils import download_models
download_models(["hey_jarvis"])
PY
```

OpenWakeWord also ships model files inside site-packages. If you want to copy those to the workspace folder, use the **correct** venv path:

```bash
cp ~/robot_waiter_ws/.venv_wake/lib/python3.10/site-packages/openwakeword/resources/models/* \
   ~/robot_waiter_ws/wakeword_models/ || true
```

Verify:

```bash
ls ~/robot_waiter_ws/wakeword_models
```

You should see `.tflite` and/or `.onnx` files (e.g., `hey_jarvis_v0.1.onnx`).

---

## Build the Package

```bash
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select waiter_wake_word --symlink-install
```

---

## Running the Node (Correct + Reliable)

### Terminal 1 — Wake Word Node

```bash
cd ~/robot_waiter_ws
source .venv_wake/bin/activate
export PYTHONNOUSERSITE=1         # prevents accidentally importing ~/.local packages
source /opt/ros/humble/setup.bash
source install/setup.bash

ros2 run waiter_wake_word wake_word_node --ros-args \
  -p wake_word_model:=hey_jarvis \
  -p model_dir:=/home/$USER/robot_waiter_ws/wakeword_models \
  -p threshold:=0.55
```

> ALSA warnings are normal on Linux and **do not indicate failure**.

### Terminal 2 — Verify Topics

```bash
ros2 topic echo /wake_word/activated
ros2 topic echo /robot/state
ros2 topic echo /led/command
```

Expected output:

```text
data: true
```

---

## ⚠️ Important: “ros2 run uses /usr/bin/python3” (Shebang issue)

Some systems/builds generate the ROS console script with:

```bash
#!/usr/bin/python3
```

That forces **system Python**, ignoring your venv, causing NumPy/SciPy conflicts.

### Option A (Recommended): rebuild using venv Python
Run the build with venv active and force Python3 executable:

```bash
cd ~/robot_waiter_ws
source .venv_wake/bin/activate
export PYTHONNOUSERSITE=1
source /opt/ros/humble/setup.bash

rm -rf build install log
colcon build --packages-select waiter_wake_word --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=$(which python3)

source install/setup.bash
head -n 1 install/waiter_wake_word/lib/waiter_wake_word/wake_word_node
```

You want the shebang to point to the venv Python.

### Option B (Quick fix): patch to `#!/usr/bin/env python3`
If you prefer env-style shebang (works **as long as venv is active**):

```bash
sed -i '1 s|^#!.*python3$|#!/usr/bin/env python3|' \
  ~/robot_waiter_ws/install/waiter_wake_word/lib/waiter_wake_word/wake_word_node
head -n 1 ~/robot_waiter_ws/install/waiter_wake_word/lib/waiter_wake_word/wake_word_node
```

Then always run with:

```bash
source ~/robot_waiter_ws/.venv_wake/bin/activate
export PYTHONNOUSERSITE=1
```

### Option C (Always works): run module with venv Python
```bash
~/robot_waiter_ws/.venv_wake/bin/python3 -m waiter_wake_word.wake_word_node --ros-args \
  -p wake_word_model:=hey_jarvis \
  -p model_dir:=/home/$USER/robot_waiter_ws/wakeword_models \
  -p threshold:=0.55
```

---

## Launch File (Optional)

```bash
ros2 launch waiter_wake_word wake_word.launch.py
```

---

## Jetson / TurtleBot4 Notes

- CPU‑only (TFLite/ONNXRuntime), no CUDA required
- Microphone device may require:

```bash
arecord -l
```

Then set:

```bash
-p mic_device_index:=N
```

---

## Known Non‑Issues (Safe to Ignore)

- ALSA `cannot open slave`
- `Unknown PCM cards`
- `Invalid field card`

These are Linux audio backend messages and do not affect detection.

---

## Responsibility Boundary

This node **only detects wake words and publishes intent**.

It does **not**:
- Perform speech‑to‑text
- Handle navigation
- Control robot motion
- Manage UI or tablet interaction

Downstream systems must subscribe to `/wake_word/activated`.

---

## Status

✔ **Complete**  
✔ **Ready for integration**

---

## Author

Wake‑Word Perception Module — Robot Waiter Project  
University of Hertfordshire
