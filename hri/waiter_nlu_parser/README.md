# waiter_nlu_parser

ROS 2 **NLU + LLM (OpenAI)** node for the **Robot Waiter** project.

This package converts **speech-to-text (STT) transcripts** into a **validated, structured order JSON** using an OpenAI GPT‑5 model, with a built‑in **clarification loop** when user intent is ambiguous.

---

## What this package does

**End‑to‑end flow**:

```
Wake Word → STT → waiter_nlu_parser → /order/json
                             ↳ clarification (if needed)
```

Given a transcript like:

> "table 3 two chicken burger no mayo and a large coke"

The node outputs:

```json
{
  "table_id": "T3",
  "items": [
    {"name": "chicken burger", "qty": 2, "mods": ["no mayo"], "size": "regular"},
    {"name": "coke", "qty": 1, "mods": [], "size": "large"}
  ],
  "notes": "",
  "needs_clarification": false,
  "clarification_question": "",
  "confidence": 0.8
}
```

If anything is missing or unclear, the node **does not guess** — it asks a clarification question instead.

---

## ROS 2 Interfaces

### Subscribed topics

| Topic | Type | Description |
|------|------|-------------|
| `/stt/text` | `std_msgs/String` | Speech‑to‑text transcript from STT node |

> The STT topic name is configurable via a ROS parameter.

---

### Published topics

| Topic | Type | Description |
|------|------|-------------|
| `/order/json` | `std_msgs/String` | Final validated order JSON |
| `/order/need_clarification` | `std_msgs/Bool` | `true` if user clarification is required |
| `/order/clarification_question` | `std_msgs/String` | Question to ask the user |
| `/order/raw_llm` | `std_msgs/String` | Raw LLM output (debugging) |

---

## Package structure

```
waiter_nlu_parser/
├── launch/
│   └── nlu_parser.launch.py
├── waiter_nlu_parser/
│   ├── __init__.py
│   ├── nlu_node.py          # Main ROS 2 node
│   ├── prompts/             # (Reserved for future prompt files)
│   └── schemas/             # (Reserved for future schema files)
├── package.xml
├── setup.py
└── README.md
```

---

## Requirements

### System
- Ubuntu 22.04
- ROS 2 **Humble**
- Python ≥ 3.10

### Python dependencies
- `openai` (official OpenAI client)

Install:
```bash
python3 -m pip install --upgrade openai
```

---

## OpenAI API setup

### 1. Create an API key

- Go to **OpenAI Dashboard → API Keys**
- Create a new key (e.g. `robot-waiter-nlu`)
- Copy the key (you will not see it again)

### 2. Export the key

```bash
export OPENAI_API_KEY="sk-xxxxxxxxxxxxxxxxxxxx"
```

Verify:
```bash
echo $OPENAI_API_KEY | wc -c
```

(Optional, recommended) Make it persistent:

```bash
nano ~/.bashrc
```
Add:
```bash
export OPENAI_API_KEY="sk-xxxxxxxxxxxxxxxxxxxx"
```
Then:
```bash
source ~/.bashrc
```

---

## Supported models

This package uses the **OpenAI Responses API**.

Recommended models:

| Model | Use case |
|-----|---------|
| `gpt-5-mini` | **Default / recommended** — best reliability |
| `gpt-5-nano` | Cheaper & faster, use after validation is stable |

> ⚠️ GPT‑5 models **do not support `temperature`**. The node is implemented accordingly.

---

## Configuration (ROS parameters)

Set in `launch/nlu_parser.launch.py` or via CLI:

| Parameter | Default | Description |
|---------|--------|-------------|
| `stt_topic` | `/stt/text` | STT transcript topic |
| `model` | `gpt-5-mini` | OpenAI model name |
| `api_key_env` | `OPENAI_API_KEY` | Environment variable holding API key |
| `menu_json_path` | `""` | Optional external menu JSON |

---

## Menu constraints

A **hard‑coded default menu** is included in `nlu_node.py`.

Only items in the menu are allowed. If the user requests an unknown item, the system asks for clarification.

To use an external menu file:

```json
{
  "items": ["chicken burger", "beef burger", "coke"],
  "modifiers": ["no mayo", "extra cheese"],
  "sizes": ["small", "regular", "large"]
}
```

Set:
```python
"menu_json_path": "/absolute/path/to/menu.json"
```

---

## Build instructions

```bash
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select waiter_nlu_parser
source install/setup.bash
```

Verify:
```bash
ros2 pkg list | grep waiter_nlu_parser
```

---

## Running the node

### Launch

```bash
ros2 launch waiter_nlu_parser nlu_parser.launch.py
```

Expected log:
```
[waiter_nlu_parser]: NLU ready. Sub: /stt/text | Pub: /order/json | model=gpt-5-mini
```

---

## Testing (without STT)

Publish a fake transcript:

```bash
ros2 topic pub --once /stt/text std_msgs/String \
"{data: 'table 3 two chicken burger no mayo and a large coke'}"
```

Observe outputs:

```bash
ros2 topic echo /order/json
ros2 topic echo /order/need_clarification
ros2 topic echo /order/clarification_question
```

---

## Clarification behaviour

If the transcript is ambiguous:

> "two burgers and a drink"

Then:
- `/order/need_clarification` → `true`
- `/order/clarification_question` → e.g. *"Which burgers and which drink?"*
- `/order/json` is **not** published

This guarantees **no hallucinated orders**.

---

## Integration notes

- Designed to plug directly into `waiter_orchestrator`
- `/order/json` is stable and schema‑validated
- Safe to run on **Jetson** (latency ~5–8s typical)

---

## Issue tracking

This package completes:

> **GitHub Issue #14 — NLU + LLM Integration**

Acceptance criteria satisfied:
- Valid order JSON ≥ 90% of tests
- Clarification triggered on ambiguity
- Works with slang/incomplete speech
- End‑to‑end STT → NLU → Order

---

## Next steps (optional)

- Swap to `gpt-5-nano` for cost reduction
- Load menu dynamically from UI module
- Multi‑turn clarification memory
- Connect `/order/json` → navigation + delivery pipeline

---

## Maintainer

Robot Waiter Project

ROS 2 Humble • OpenAI GPT‑5 • Structured NLU

