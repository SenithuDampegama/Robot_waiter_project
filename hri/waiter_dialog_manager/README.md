# waiter_dialog_manager (ROS 2 Humble)

State-machine-based **Dialog Manager** for the Robot Waiter project.

It glues together:
- Wake word trigger
- Speech-to-text (STT)
- NLU/LLM order parsing
- Confirmation + error handling
- Publishing a final order to the orchestrator
- Optional UI feedback topics (robot state, LEDs/OLED)

This package is designed to be **simple, testable, and composable**: you can run it alone with mocked topics, or run it alongside the real wake-word + STT + NLU nodes.

---

## What this node does

The dialog manager is a finite state machine (FSM) that drives the customer interaction.

### High-level states
- **GREETING / IDLE**: ready to start
- **WAIT_WAKE**: waiting for wake word pulse
- **LISTEN_ORDER**: waiting for STT text (the order utterance)
- **WAIT_NLU**: waiting for an order JSON from the NLU parser
- **CONFIRM_WAIT_WAKE**: asks user to say wake word again before confirmation (prevents accidental confirmations)
- **CONFIRM_LISTEN**: waits for STT “yes/no”
- **SEND_TO_ORCH**: publishes final order to `/orders`
- **ERROR / RECOVERY**: timeouts, invalid order, retry loop

### Expected ROS topics (current project conventions)
**Inputs**
- `/wake_word/activated` (`std_msgs/Bool`) — wake word detection pulse
- `/stt/text` (`std_msgs/String`) — transcript text from STT
- `/order/json` (`std_msgs/String`) — parsed order JSON from `waiter_nlu_parser`
- `/order/need_clarification` (`std_msgs/Bool`) — NLU says clarification required
- `/order/clarification_question` (`std_msgs/String`) — question to ask the user

**Outputs**
- `/orders` (`std_msgs/String`) — final order JSON sent to the orchestrator

**Optional UI outputs (recommended)**
- `/robot/state` (`std_msgs/String`) — e.g., `idle`, `listening`, `confirming`
- `/led/command` (`std_msgs/String`) — e.g., `listening_mode`, `idle_mode`

> Note: The dialog manager **does not call the LLM**. It waits for `waiter_nlu_parser` to publish `/order/json`.

---

## Repository location

This package lives in:

```bash
robot_waiter_ws/src/waiter_dialog_manager
```

---

## Dependencies

- ROS 2 Humble
- `rclpy`
- `std_msgs`

Build system:
- `colcon`

---

## Build

From the workspace root:

```bash
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select waiter_dialog_manager
source install/setup.bash
```

---

## Run (Dialog Manager only)

```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_dialog_manager dialog_manager.launch.py
```

You should see logs like:
- `DialogManager up...`
- `Waiting for wake word.`

---

## Run (full speech pipeline)

### Terminal 1 — Wake word
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_wake_word wake_word.launch.py
```

### Terminal 2 — Speech-to-text
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_speech_to_text speech_to_text.launch.py
```

### Terminal 3 — NLU/LLM parser
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_nlu_parser nlu_parser.launch.py
```

### Terminal 4 — Dialog Manager
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_dialog_manager dialog_manager.launch.py
```

### Terminal 5 — Watch final orders
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 topic echo /orders
```

Now speak:
> "Hey Jarvis" (wake word)
> "My table id is 3 and I want a coke" (order)

Dialog manager should drive the flow, and `/orders` should publish a JSON order.

---

## Quick smoke test (no microphone required)

You can test the FSM using topic publishes.

### 1) Start dialog manager
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash
ros2 launch waiter_dialog_manager dialog_manager.launch.py
```

### 2) In another terminal, simulate wake + STT + NLU
```bash
source /opt/ros/humble/setup.bash
source ~/robot_waiter_ws/install/setup.bash

# wake word pulse (moves to LISTEN_ORDER)
ros2 topic pub --once /wake_word/activated std_msgs/Bool "{data: true}"

# order transcript (moves to WAIT_NLU)
ros2 topic pub --once /stt/text std_msgs/String "{data: 'my table id is 3 and i want a coke'}"

# NLU outputs (moves to confirm states)
ros2 topic pub --once /order/need_clarification std_msgs/Bool "{data: false}"
ros2 topic pub --once /order/json std_msgs/String "{data: '{\"table_id\":\"3\",\"items\":[{\"name\":\"coke\",\"qty\":1}],\"notes\":\"\",\"needs_clarification\":false}'}"

# confirm requires wake again then yes/no
ros2 topic pub --once /wake_word/activated std_msgs/Bool "{data: true}"
ros2 topic pub --once /stt/text std_msgs/String "{data: 'yes'}"
```

### 3) Watch output
```bash
ros2 topic echo /orders
```

You should see the JSON order published.

---

## Common issues

### 1) `Package 'waiter_dialog_manager' not found`
You didn’t source the workspace, or you built only other packages.

Fix:
```bash
cd ~/robot_waiter_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select waiter_dialog_manager
source install/setup.bash
```

### 2) Launch file not found
If `ros2 launch ... dialog_manager.launch.py` says it can’t find the file, the launch file wasn’t installed.

Fix: ensure `setup.py` installs launch files:
- `data_files` includes `('share/<pkg>/launch', glob('launch/*.launch.py'))`

Then rebuild.

### 3) Dialog manager "does nothing"
It will appear idle if:
- no wake word pulses arrive on `/wake_word/activated`
- no STT arrives on `/stt/text`
- no NLU output arrives on `/order/json`

Use `ros2 topic echo` to verify each input.

---

## Notes on design

- **Dialog manager is orchestration logic**, not the parser.
- `waiter_nlu_parser` is the only component that calls the LLM.
- The dialog manager should remain lightweight and robust under noisy audio.

---

## License

MIT (recommended). Update `package.xml` and `setup.py` accordingly.

