#!/usr/bin/env python3
"""
Wake Word Node (ROS 2 Humble) - Robot Waiter

Publishes:
  /wake_word/activated (std_msgs/Bool)    -> True pulse when wake word detected
  /robot/state         (std_msgs/String) -> "listening" then "idle"
  /led/command         (std_msgs/String) -> "listening_mode" then "idle_mode"
  /wake_word/score     (std_msgs/Float32)-> smoothed confidence score

Requires:
  pip install openwakeword pyaudio numpy
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np
import pyaudio
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32

from openwakeword.model import Model


@dataclass
class Stats:
    detections: int = 0
    suppressed_cooldown: int = 0
    audio_errors: int = 0
    avg_latency_ms_ema: Optional[float] = None
    last_score: float = 0.0


class WakeWordNode(Node):
    def __init__(self):
        super().__init__("wake_word_node")

        # -----------------------
        # Parameters
        # -----------------------
        self.declare_parameter("wake_word_model", "hey_jarvis")
        self.declare_parameter("model_dir", "")
        self.declare_parameter("threshold", 0.40)
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("channels", 1)
        self.declare_parameter("chunk_size", 1280)  # 80 ms
        self.declare_parameter("mic_device_index", -1)
        self.declare_parameter("cooldown_s", 2.0)
        self.declare_parameter("listening_hold_s", 5.0)
        self.declare_parameter("score_smoothing_alpha", 0.35)
        self.declare_parameter("log_period_s", 10.0)

        self.wake_word_model = self.get_parameter("wake_word_model").value
        self.model_dir = self.get_parameter("model_dir").value
        self.threshold = float(self.get_parameter("threshold").value)
        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.channels = int(self.get_parameter("channels").value)
        self.chunk_size = int(self.get_parameter("chunk_size").value)
        self.mic_device_index = int(self.get_parameter("mic_device_index").value)
        self.cooldown_s = float(self.get_parameter("cooldown_s").value)
        self.listening_hold_s = float(self.get_parameter("listening_hold_s").value)
        self.score_alpha = float(self.get_parameter("score_smoothing_alpha").value)
        self.log_period_s = float(self.get_parameter("log_period_s").value)

        # -----------------------
        # Publishers
        # -----------------------
        self.pub_activated = self.create_publisher(Bool, "/wake_word/activated", 10)
        self.pub_state = self.create_publisher(String, "/robot/state", 10)
        self.pub_led = self.create_publisher(String, "/led/command", 10)
        self.pub_score = self.create_publisher(Float32, "/wake_word/score", 10)

        # -----------------------
        # Load wakeword model
        # -----------------------
        try:
            if self.model_dir:
                model_path = f"{self.model_dir}/{self.wake_word_model}_v0.1.tflite"
                self.get_logger().info(f"Loading wakeword model from file: {model_path}")
                self.model = Model(wakeword_models=[model_path])
            else:
                self.model = Model(wakeword_models=[self.wake_word_model])

            self.get_logger().info("Wakeword model loaded OK")

        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")
            self.get_logger().warn("Falling back to default OpenWakeWord model set")
            self.model = Model()

        # -----------------------
        # Audio
        # -----------------------
        self.audio = pyaudio.PyAudio()
        self.stream = None

        self.running = True
        self._last_trigger_t = 0.0
        self._score_ema = 0.0
        self._lock = threading.Lock()
        self.stats = Stats()

        self._open_stream()

        self.audio_thread = threading.Thread(target=self._audio_loop, daemon=True)
        self.audio_thread.start()

        self.create_timer(self.log_period_s, self._log_stats)

        self.get_logger().info("WakeWordNode running (listening to mic)...")

    # --------------------------------------------------

    def _open_stream(self):
        try:
            kwargs = dict(
                format=pyaudio.paInt16,
                channels=self.channels,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=self.chunk_size,
            )
            if self.mic_device_index >= 0:
                kwargs["input_device_index"] = self.mic_device_index

            self.stream = self.audio.open(**kwargs)
            self.get_logger().info("Microphone stream opened.")

        except Exception as e:
            self.get_logger().error(f"Microphone open failed: {e}")
            self.stream = None

    # --------------------------------------------------

    def _audio_loop(self):
        if self.stream is None:
            return

        while self.running:
            try:
                audio_bytes = self.stream.read(
                    self.chunk_size, exception_on_overflow=False
                )
                audio_i16 = np.frombuffer(audio_bytes, dtype=np.int16)

                t0 = time.time()
                pred = self.model.predict(audio_i16)
                latency_ms = (time.time() - t0) * 1000.0

                with self._lock:
                    if self.stats.avg_latency_ms_ema is None:
                        self.stats.avg_latency_ms_ema = latency_ms
                    else:
                        self.stats.avg_latency_ms_ema = (
                            0.15 * latency_ms
                            + 0.85 * self.stats.avg_latency_ms_ema
                        )

                best_word, best_score = None, 0.0
                for word, score in pred.items():
                    s = float(score)
                    if s > best_score:
                        best_score = s
                        best_word = word

                self._score_ema = (
                    self.score_alpha * best_score
                    + (1.0 - self.score_alpha) * self._score_ema
                )

                self.pub_score.publish(Float32(data=float(self._score_ema)))
                self.stats.last_score = float(self._score_ema)

                if (
                    best_word
                    and self._score_ema >= self.threshold
                    and time.time() - self._last_trigger_t >= self.cooldown_s
                ):
                    self._last_trigger_t = time.time()
                    self._on_detect(best_word, self._score_ema, latency_ms)

            except Exception as e:
                self.stats.audio_errors += 1
                self.get_logger().warn(f"Audio error: {e}")
                time.sleep(0.1)

    # --------------------------------------------------

    def _on_detect(self, word: str, confidence: float, latency_ms: float):
        self.stats.detections += 1

        self.get_logger().info(
            f"Wake word DETECTED: '{word}' | conf={confidence:.2f} | latency={latency_ms:.1f} ms"
        )

        self.pub_activated.publish(Bool(data=True))
        self._set_listening(True)

        threading.Timer(
            self.listening_hold_s, self._set_listening, args=(False,)
        ).start()

    # --------------------------------------------------

    def _set_listening(self, listening: bool):
        if listening:
            self.pub_state.publish(String(data="listening"))
            self.pub_led.publish(String(data="listening_mode"))
        else:
            self.pub_state.publish(String(data="idle"))
            self.pub_led.publish(String(data="idle_mode"))
            self.get_logger().info("Returned to idle state.")

    # --------------------------------------------------

    def _log_stats(self):
        lat = self.stats.avg_latency_ms_ema or 0.0
        self.get_logger().info(
            f"[wakeword stats] detections={self.stats.detections} | "
            f"suppressed_cooldown={self.stats.suppressed_cooldown} | "
            f"audio_errors={self.stats.audio_errors} | "
            f"latency_ema_ms={lat:.1f} | last_score={self.stats.last_score:.2f} | "
            f"threshold={self.threshold:.2f}"
        )

    # --------------------------------------------------

    def shutdown(self):
        self.running = False
        try:
            if self.stream:
                self.stream.stop_stream()
                self.stream.close()
            self.audio.terminate()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = WakeWordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # stop audio + threads
        try:
            node._on_shutdown()
        except Exception:
            pass

        try:
            node.destroy_node()
        except Exception:
            pass

        # shutdown ROS only if not already shutdown
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
