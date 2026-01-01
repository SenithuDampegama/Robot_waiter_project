# waiter_speech_to_text

Offline **Speech-to-Text (STT)** package for the **Robot Waiter** project (ROS 2 Humble).

This package converts spoken customer input into text **only after a wake word is detected**, and publishes the transcript, confidence score, and timestamp for downstream dialog / NLU modules.

It is designed to be:
- **Offline** (no internet / cloud dependency)
- **Low-latency** (CPU-only, works on Jetson)
- **ROS-native** (proper use of `share/`, parameters, topics)
- **Reproducible** (model packaged with the node)

---

## Architecture Overview

```
Wake Word Node
    │  (/wake_word/activated)
    ▼
Speech-to-Text Node  (this package)
    │
    ├─ /stt/text        (std_msgs/String)
    ├─ /stt/confidence  (std_msgs/Float32)
    └─ /stt/timestamp   (builtin_interfaces/Time)
            │
            ▼
Dialog Manager / NLU
```

The STT node **does nothing** until the wake word topic publishes `true`.

---

## Package Structure

```
waiter_speech_to_text/
├── waiter_speech_to_text/
│   └── speech_to_text_node.py
├── share/
│   └── models/
│       └── vosk-model-small-en-us-0.15/
│           ├── am/
│           ├── conf/
│           ├── graph/
│           ├── ivector/
│           └── README
├── resource/
├── package.xml
├── setup.py
└── README.md   ← you are here
```

The Vosk model is installed under the package **share directory** so it is available in `install/` on Jetson and other machines.

---

## Dependencies

### System (Ubuntu 22.04 / ROS 2 Humble)

```bash
sudo apt update
sudo apt install -y portaudio19-dev python3-pyaudio sox
```

### Python

```bash
python3 -m pip install vosk sounddevice numpy
```

---

## Build Instructions

From your ROS 2 workspace:

```bash
cd ~/robot_waiter_ws
colcon build --packages-select waiter_speech_to_text --symlink-install
source install/setup.bash
```

Verify the model was installed correctly:

```bash
ls install/waiter_speech_to_text/share/waiter_speech_to_text/models/vosk-model-small-en-us-0.15
```

You should see directories like `am/ conf/ graph/ ivector/`.

---

## Running the Node

### 1️⃣ Start the STT node

```bash
ros2 run waiter_speech_to_text speech_to_text_node
```

Expected output:
```
[INFO] Vosk model loaded
[INFO] Subscribed to wake topic: /wake_word/activated
[INFO] Speech-to-Text node ready
```

---

### 2️⃣ Trigger via Wake Word

If you already have a wake-word node running, nothing else is needed.

For manual testing:

```bash
ros2 topic pub /wake_word/activated std_msgs/Bool "data: true"
```

Speak clearly after triggering the wake event.

---

### 3️⃣ Observe Output Topics

```bash
ros2 topic echo /stt/text
ros2 topic echo /stt/confidence
ros2 topic echo /stt/timestamp
```

Example:
```
data: "can i have a chicken please"

confidence: 0.89

timestamp:
  sec: 1767223707
  nanosec: 87179107
```

---

## ROS Parameters

| Parameter | Default | Description |
|---------|--------|------------|
| `wake_topic` | `/wake_word/activated` | Wake-word trigger topic |
| `text_topic` | `/stt/text` | Transcript output |
| `conf_topic` | `/stt/confidence` | Confidence output |
| `ts_topic` | `/stt/timestamp` | Timestamp output |
| `sample_rate` | `16000` | Audio sample rate |
| `max_record_seconds` | `8.0` | Max listening duration |
| `silence_rms_thresh` | `0.008` | Silence detection threshold |
| `silence_seconds` | `1.0` | Silence timeout |
| `min_speech_seconds` | `0.3` | Minimum speech length |
| `model_dir` | *(auto)* | Vosk model path (from share directory) |

Example override:

```bash
ros2 run waiter_speech_to_text speech_to_text_node --ros-args \
  -p silence_rms_thresh:=0.006 \
  -p silence_seconds:=1.4
```

---

## Notes for Jetson Deployment

- Fully **CPU-only** (no CUDA / TensorRT needed)
- Tested on x86_64; suitable for Jetson Orin Nano / Xavier
- No internet required
- Uses standard ALSA / PulseAudio input

If multiple microphones are present, ensure the correct input device is selected at OS level.

---

## Known Limitations

- Small Vosk model prioritizes speed over accuracy
- Strong accents or heavy noise may reduce confidence
- No language switching (English only)

These are acceptable for restaurant dialog use and can be upgraded later.

---

## Future Improvements (Optional)

- Confidence-based rejection (e.g., ignore `< 0.6`)
- Cooldown after transcript to avoid repeated triggers
- Replace Vosk with Whisper / TensorRT in future
- Launch file integration with wake word + dialog manager

---

## Status

**Issue #12 – Speech-to-Text Integration: COMPLETE** ✅

This package is production-ready for the Robot Waiter project and safe to push to GitHub.
