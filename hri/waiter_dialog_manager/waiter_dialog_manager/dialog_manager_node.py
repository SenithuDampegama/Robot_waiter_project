#!/usr/bin/env python3
"""
Dialog Manager Node (ROS 2 Humble) - Robot Waiter

FSM states:
  GREETING -> WAIT_WAKE -> LISTENING -> WAIT_NLU -> (CLARIFY or CONFIRM) -> SEND_TO_ORCH -> END -> WAIT_WAKE

Key design constraint:
  STT only records on /wake_word/activated, so confirmation/clarification requires wake-word again.
"""

from enum import Enum, auto
import time
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, String


class State(Enum):
    GREETING = auto()
    WAIT_WAKE = auto()
    LISTENING = auto()
    WAIT_NLU = auto()
    CLARIFY_WAIT_WAKE = auto()
    CLARIFY_LISTENING = auto()
    CLARIFY_WAIT_NLU = auto()
    CONFIRM_WAIT_WAKE = auto()
    CONFIRM_LISTENING = auto()
    SEND_TO_ORCH = auto()
    END = auto()


def _now_s() -> float:
    return time.time()


def _clean_text(t: str) -> str:
    return " ".join((t or "").strip().lower().split())


def _is_yes(t: str) -> bool:
    t = _clean_text(t)
    return t in {"yes", "yeah", "yep", "confirm", "correct", "that's right", "right"} or t.startswith("yes ")


def _is_no(t: str) -> bool:
    t = _clean_text(t)
    return t in {"no", "nope", "nah", "incorrect", "wrong"} or t.startswith("no ")


def _is_cancel(t: str) -> bool:
    t = _clean_text(t)
    return any(k in t for k in ["cancel", "stop", "never mind", "nevermind", "exit", "quit"])


class DialogManager(Node):
    def __init__(self):
        super().__init__("dialog_manager")

        # ---------- Parameters (topics) ----------
        self.declare_parameter("topics.wake", "/wake_word/activated")
        self.declare_parameter("topics.stt_text", "/stt/text")
        self.declare_parameter("topics.order_json", "/order/json")
        self.declare_parameter("topics.need_clarification", "/order/need_clarification")
        self.declare_parameter("topics.clarification_question", "/order/clarification_question")

        self.declare_parameter("topics.orders_out", "/orders")
        self.declare_parameter("topics.robot_state", "/robot/state")
        self.declare_parameter("topics.led", "/led/command")
        self.declare_parameter("topics.oled", "/oled/text")  # optional, safe even if nobody subscribes

        # ---------- Parameters (timing) ----------
        self.declare_parameter("greeting_seconds", 0.5)
        self.declare_parameter("stt_timeout_s", 10.0)           # wait for /stt/text after wake
        self.declare_parameter("nlu_timeout_s", 12.0)           # wait for /order/* after stt
        self.declare_parameter("max_retries", 2)
        self.declare_parameter("idle_timeout_s", 60.0)          # overall idle watchdog
        self.declare_parameter("tick_hz", 10.0)

        # Load params
        self.t_wake = self.get_parameter("topics.wake").value
        self.t_stt = self.get_parameter("topics.stt_text").value
        self.t_order_json = self.get_parameter("topics.order_json").value
        self.t_need = self.get_parameter("topics.need_clarification").value
        self.t_q = self.get_parameter("topics.clarification_question").value

        self.t_orders_out = self.get_parameter("topics.orders_out").value
        self.t_robot_state = self.get_parameter("topics.robot_state").value
        self.t_led = self.get_parameter("topics.led").value
        self.t_oled = self.get_parameter("topics.oled").value

        self.greeting_seconds = float(self.get_parameter("greeting_seconds").value)
        self.stt_timeout_s = float(self.get_parameter("stt_timeout_s").value)
        self.nlu_timeout_s = float(self.get_parameter("nlu_timeout_s").value)
        self.max_retries = int(self.get_parameter("max_retries").value)
        self.idle_timeout_s = float(self.get_parameter("idle_timeout_s").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)

        # ---------- Publishers ----------
        self.pub_orders = self.create_publisher(String, self.t_orders_out, 10)
        self.pub_state = self.create_publisher(String, self.t_robot_state, 10)
        self.pub_led = self.create_publisher(String, self.t_led, 10)
        self.pub_oled = self.create_publisher(String, self.t_oled, 10)

        # ---------- Subscriptions ----------
        self.create_subscription(Bool, self.t_wake, self.on_wake, 10)
        self.create_subscription(String, self.t_stt, self.on_stt, 10)
        self.create_subscription(String, self.t_order_json, self.on_order_json, 10)
        self.create_subscription(Bool, self.t_need, self.on_need_clarification, 10)
        self.create_subscription(String, self.t_q, self.on_clarification_question, 10)

        # ---------- FSM Data ----------
        self.state = State.GREETING
        self.state_entered_at = _now_s()
        self.last_activity_at = _now_s()

        self.retry_count = 0

        self.last_wake_pulse_at: Optional[float] = None
        self.last_stt_text: Optional[str] = None

        self.pending_order_json: Optional[str] = None
        self.need_clarification: Optional[bool] = None
        self.clarification_question: Optional[str] = None

        # Flags to know when "new data arrived" during a state
        self._stt_arrived = False
        self._order_arrived = False
        self._need_arrived = False
        self._q_arrived = False

        # Tick loop
        self.timer = self.create_timer(1.0 / max(self.tick_hz, 1.0), self.tick)

        self._enter(State.GREETING, say="Hello! Say the wake word when you're ready to order.", led="idle_mode")

        self.get_logger().info(
            "DialogManager up. Using topics:"
            f" wake={self.t_wake}, stt={self.t_stt}, order_json={self.t_order_json}, orders_out={self.t_orders_out}"
        )

    # ----------------- Helpers -----------------
    def _publish_state(self, s: str):
        self.pub_state.publish(String(data=s))

    def _publish_led(self, cmd: str):
        self.pub_led.publish(String(data=cmd))

    def _say(self, text: str):
        # For now we publish to OLED (and log). Later you can hook TTS here.
        self.pub_oled.publish(String(data=text))
        self.get_logger().info(f"[SAY] {text}")

    def _enter(self, new_state: State, say: Optional[str] = None, led: Optional[str] = None, robot_state: Optional[str] = None):
        self.state = new_state
        self.state_entered_at = _now_s()
        self.last_activity_at = _now_s()

        # reset per-state arrival flags
        self._stt_arrived = False
        self._order_arrived = False
        self._need_arrived = False
        self._q_arrived = False

        if robot_state is None:
            robot_state = new_state.name.lower()

        self._publish_state(robot_state)

        if led is not None:
            self._publish_led(led)

        if say:
            self._say(say)

    def _timed_out(self, timeout_s: float) -> bool:
        return (_now_s() - self.state_entered_at) > timeout_s

    # ----------------- Callbacks -----------------
    def on_wake(self, msg: Bool):
        if msg.data:
            self.last_wake_pulse_at = _now_s()
            self.last_activity_at = _now_s()

            # State transitions that use wake word
            if self.state in {State.WAIT_WAKE, State.CONFIRM_WAIT_WAKE, State.CLARIFY_WAIT_WAKE}:
                if self.state == State.WAIT_WAKE:
                    self._enter(State.LISTENING, say="Listening. Please tell me your order.", led="listening_mode")
                elif self.state == State.CONFIRM_WAIT_WAKE:
                    self._enter(State.CONFIRM_LISTENING, say="Listening. Say yes to confirm, or no to change.", led="listening_mode")
                elif self.state == State.CLARIFY_WAIT_WAKE:
                    self._enter(State.CLARIFY_LISTENING, say="Listening. Please answer the clarification.", led="listening_mode")

    def on_stt(self, msg: String):
        txt = (msg.data or "").strip()
        if not txt:
            return
        self.last_stt_text = txt
        self._stt_arrived = True
        self.last_activity_at = _now_s()
        self.get_logger().info(f"[STT] {txt}")

    def on_order_json(self, msg: String):
        data = (msg.data or "").strip()
        if not data:
            return
        self.pending_order_json = data
        self._order_arrived = True
        self.last_activity_at = _now_s()
        self.get_logger().info("[NLU] order/json received")

    def on_need_clarification(self, msg: Bool):
        self.need_clarification = bool(msg.data)
        self._need_arrived = True
        self.last_activity_at = _now_s()
        self.get_logger().info(f"[NLU] need_clarification={self.need_clarification}")

    def on_clarification_question(self, msg: String):
        q = (msg.data or "").strip()
        if not q:
            return
        self.clarification_question = q
        self._q_arrived = True
        self.last_activity_at = _now_s()
        self.get_logger().info(f"[NLU] clarification_question: {q}")

    # ----------------- FSM Tick -----------------
    def tick(self):
        # Global idle watchdog
        if (_now_s() - self.last_activity_at) > self.idle_timeout_s and self.state != State.WAIT_WAKE:
            self._enter(State.WAIT_WAKE, say="Session timed out. Say the wake word when you're ready.", led="idle_mode")
            self._clear_buffers()
            return

        if self.state == State.GREETING:
            if self._timed_out(self.greeting_seconds):
                self._enter(State.WAIT_WAKE, say="Waiting for wake word.", led="idle_mode")
            return

        # ---------------- WAIT_WAKE ----------------
        if self.state == State.WAIT_WAKE:
            return  # wait for on_wake to move to LISTENING

        # ---------------- LISTENING ----------------
        if self.state == State.LISTENING:
            # Wait for STT text (wake word already triggered STT)
            if self._stt_arrived:
                if _is_cancel(self.last_stt_text or ""):
                    self._enter(State.END, say="Okay, cancelled.", led="idle_mode")
                else:
                    # Now wait for NLU outputs driven by STT
                    self._enter(State.WAIT_NLU, say="Processing your order...", led="thinking_mode", robot_state="parsing_order")
            elif self._timed_out(self.stt_timeout_s):
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self._enter(State.END, say="Sorry, I didn't catch that. Please try again later.", led="idle_mode")
                else:
                    self._enter(State.WAIT_WAKE, say="I didn't hear anything. Say the wake word and try again.", led="idle_mode")
            return

        # ---------------- WAIT_NLU ----------------
        if self.state == State.WAIT_NLU:
            # We consider NLU ready when order_json arrived AND need_clarification arrived (or timeout fallback)
            if self._order_arrived and self._need_arrived:
                if self.need_clarification:
                    q = self.clarification_question or "I need more details. Please clarify."
                    self._enter(
                        State.CLARIFY_WAIT_WAKE,
                        say=f"{q} Say the wake word, then answer.",
                        led="idle_mode",
                        robot_state="clarifying",
                    )
                else:
                    # Go to confirmation
                    self._enter(
                        State.CONFIRM_WAIT_WAKE,
                        say="I understood your order. Say the wake word, then say YES to confirm or NO to change.",
                        led="idle_mode",
                        robot_state="confirming_order",
                    )
            elif self._timed_out(self.nlu_timeout_s):
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self._enter(State.END, say="Sorry, I couldn't process the order.", led="idle_mode")
                else:
                    self._enter(State.WAIT_WAKE, say="I couldn't process that. Say the wake word and try again.", led="idle_mode")
                    self._clear_buffers()
            return

        # ---------------- CLARIFY ----------------
        if self.state == State.CLARIFY_WAIT_WAKE:
            return

        if self.state == State.CLARIFY_LISTENING:
            if self._stt_arrived:
                if _is_cancel(self.last_stt_text or ""):
                    self._enter(State.END, say="Okay, cancelled.", led="idle_mode")
                else:
                    self._enter(State.CLARIFY_WAIT_NLU, say="Got it. Processing...", led="thinking_mode", robot_state="parsing_order")
            elif self._timed_out(self.stt_timeout_s):
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self._enter(State.END, say="Sorry, I didn't catch that.", led="idle_mode")
                else:
                    self._enter(State.CLARIFY_WAIT_WAKE, say="I didn't hear an answer. Say the wake word and try again.", led="idle_mode")
            return

        if self.state == State.CLARIFY_WAIT_NLU:
            if self._order_arrived and self._need_arrived:
                if self.need_clarification:
                    q = self.clarification_question or "I still need more details. Please clarify."
                    self._enter(State.CLARIFY_WAIT_WAKE, say=f"{q} Say the wake word, then answer.", led="idle_mode")
                else:
                    self._enter(
                        State.CONFIRM_WAIT_WAKE,
                        say="Thanks. Say the wake word, then say YES to confirm or NO to change.",
                        led="idle_mode",
                        robot_state="confirming_order",
                    )
            elif self._timed_out(self.nlu_timeout_s):
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self._enter(State.END, say="Sorry, I couldn't process the clarification.", led="idle_mode")
                else:
                    self._enter(State.CLARIFY_WAIT_WAKE, say="I couldn't process that. Say the wake word and try again.", led="idle_mode")
                    self._clear_buffers()
            return

        # ---------------- CONFIRM ----------------
        if self.state == State.CONFIRM_WAIT_WAKE:
            return

        if self.state == State.CONFIRM_LISTENING:
            if self._stt_arrived:
                t = self.last_stt_text or ""
                if _is_cancel(t):
                    self._enter(State.END, say="Okay, cancelled.", led="idle_mode")
                elif _is_yes(t):
                    self._enter(State.SEND_TO_ORCH, say="Confirmed. Sending your order.", led="thinking_mode", robot_state="passing_to_orchestrator")
                elif _is_no(t):
                    self._clear_buffers(keep_order=False)
                    self._enter(State.WAIT_WAKE, say="Okay. Say the wake word and tell me the order again.", led="idle_mode")
                else:
                    # Not understood -> ask again
                    self._enter(State.CONFIRM_WAIT_WAKE, say="Please say YES to confirm or NO to change. Use the wake word first.", led="idle_mode")
            elif self._timed_out(self.stt_timeout_s):
                self.retry_count += 1
                if self.retry_count > self.max_retries:
                    self._enter(State.END, say="Sorry, confirmation timed out.", led="idle_mode")
                else:
                    self._enter(State.CONFIRM_WAIT_WAKE, say="I didn't hear confirmation. Say the wake word and answer YES or NO.", led="idle_mode")
            return

        # ---------------- SEND_TO_ORCH ----------------
        if self.state == State.SEND_TO_ORCH:
            if not self.pending_order_json:
                self._enter(State.END, say="Error: no order to send.", led="idle_mode")
                return

            self.pub_orders.publish(String(data=self.pending_order_json))
            self.get_logger().info("[ORCH] Published order to /orders")

            self._enter(State.END, say="Order sent. Thank you!", led="idle_mode")
            return

        # ---------------- END ----------------
        if self.state == State.END:
            # reset for next customer
            self._clear_buffers()
            self.retry_count = 0
            self._enter(State.WAIT_WAKE, say="Waiting for wake word.", led="idle_mode")
            return

    def _clear_buffers(self, keep_order: bool = False):
        self.last_stt_text = None
        self._stt_arrived = False

        if not keep_order:
            self.pending_order_json = None
            self.need_clarification = None
            self.clarification_question = None

        self._order_arrived = False
        self._need_arrived = False
        self._q_arrived = False


def main():
    rclpy.init()
    node = DialogManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
