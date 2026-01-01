#!/usr/bin/env python3
import json
import queue
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float32
from builtin_interfaces.msg import Time as TimeMsg

import sounddevice as sd
from vosk import Model, KaldiRecognizer
from ament_index_python.packages import get_package_share_directory


class SpeechToTextNode(Node):
    """
    Offline Speech-to-Text node (Vosk), gated by wake word.

    Subscribes:
      /wake_word/activated (std_msgs/Bool)

    Publishes:
      /stt/text        (std_msgs/String)
      /stt/confidence  (std_msgs/Float32)
      /stt/timestamp   (builtin_interfaces/Time)
    """

    def __init__(self):
        super().__init__("speech_to_text_node")

        # Topics
        self.declare_parameter("wake_topic", "/wake_word/activated")
        self.declare_parameter("text_topic", "/stt/text")
        self.declare_parameter("conf_topic", "/stt/confidence")
        self.declare_parameter("ts_topic", "/stt/timestamp")

        # Audio/STT params
        self.declare_parameter("sample_rate", 16000)
        self.declare_parameter("max_record_seconds", 8.0)
        self.declare_parameter("silence_rms_thresh", 0.008)
        self.declare_parameter("silence_seconds", 2.5)
        self.declare_parameter("min_speech_seconds", 0.3)

        # Default model path from installed package share
        pkg_share = Path(get_package_share_directory("waiter_speech_to_text"))
        default_model_dir = pkg_share / "models" / "vosk-model-small-en-us-0.15"
        self.declare_parameter("model_dir", str(default_model_dir))

        # Read params
        self.wake_topic = self.get_parameter("wake_topic").value
        self.text_topic = self.get_parameter("text_topic").value
        self.conf_topic = self.get_parameter("conf_topic").value
        self.ts_topic = self.get_parameter("ts_topic").value

        self.sample_rate = int(self.get_parameter("sample_rate").value)
        self.max_record_seconds = float(self.get_parameter("max_record_seconds").value)
        self.silence_rms_thresh = float(self.get_parameter("silence_rms_thresh").value)
        self.silence_seconds = float(self.get_parameter("silence_seconds").value)
        self.min_speech_seconds = float(self.get_parameter("min_speech_seconds").value)

        model_dir = Path(self.get_parameter("model_dir").value)

        # Publishers
        self.pub_text = self.create_publisher(String, self.text_topic, 10)
        self.pub_conf = self.create_publisher(Float32, self.conf_topic, 10)
        self.pub_ts = self.create_publisher(TimeMsg, self.ts_topic, 10)

        # If model not found in install share, fallback to source tree (dev convenience)
        if not model_dir.exists():
            fallback = Path.home() / "robot_waiter_ws" / "src" / "waiter_speech_to_text" / "share" / "models" / "vosk-model-small-en-us-0.15"
            self.get_logger().warn(f"Model not found at installed share path: {model_dir}")
            self.get_logger().warn(f"Trying fallback source path: {fallback}")
            model_dir = fallback

        self.get_logger().info(f"Loading Vosk model from: {model_dir}")

        required_dirs = ["am", "conf", "graph", "ivector"]
        if not model_dir.exists() or any(not (model_dir / d).exists() for d in required_dirs):
            raise RuntimeError(
                f"Vosk model dir invalid: {model_dir}\n"
                f"Expected subdirs: {required_dirs}\n"
                f"Fix: ensure model is in src/waiter_speech_to_text/share/models/... and setup.py installs it."
            )

        self.model = Model(str(model_dir))
        self.get_logger().info("Vosk model loaded ✅")

        # Wake gating
        self.busy = False
        self.create_subscription(Bool, self.wake_topic, self.on_wake, 10)

        self.get_logger().info(f"Subscribed to wake topic: {self.wake_topic}")
        self.get_logger().info("Speech-to-Text node ready.")

    @staticmethod
    def rms(x: np.ndarray) -> float:
        return float(np.sqrt(np.mean(np.square(x))))

    def on_wake(self, msg: Bool):
        if not msg.data:
            return
        if self.busy:
            self.get_logger().warn("Wake received while busy; ignoring.")
            return

        self.busy = True
        self.get_logger().info("Wake word detected → listening")

        try:
            text, conf = self.record_and_transcribe()
            if text:
                self.publish(text, conf)
            else:
                self.get_logger().warn("No valid speech detected (silence/too short).")
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"STT failure: {e}")
        finally:
            self.busy = False

    def record_and_transcribe(self):
        q = queue.Queue()
        rec = KaldiRecognizer(self.model, self.sample_rate)
        rec.SetWords(True)

        start = time.time()
        last_voice = None
        speech_started = False

        def callback(indata, frames, time_info, status):  # noqa: ARG001
            if status:
                self.get_logger().warn(str(status))
            q.put(indata.copy())

        with sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype="float32",
            callback=callback,
        ):
            while True:
                audio = q.get()
                level = self.rms(audio)
                now = time.time()

                if level > self.silence_rms_thresh:
                    if not speech_started:
                        speech_started = True
                    last_voice = now

                # float32 [-1,1] -> int16 bytes
                pcm16 = (np.clip(audio[:, 0], -1.0, 1.0) * 32767).astype(np.int16).tobytes()
                rec.AcceptWaveform(pcm16)

                if (now - start) > self.max_record_seconds:
                    break

                if speech_started and last_voice and (now - last_voice) > self.silence_seconds:
                    break

        if not speech_started:
            return "", 0.0

        speech_dur = (last_voice - start) if last_voice else 0.0
        if speech_dur < self.min_speech_seconds:
            return "", 0.0

        final = json.loads(rec.FinalResult() or "{}")
        text = (final.get("text") or "").strip()

        conf = 1.0
        if isinstance(final.get("result"), list) and final["result"]:
            confs = [w.get("conf", 1.0) for w in final["result"] if isinstance(w, dict)]
            if confs:
                conf = float(sum(confs) / len(confs))

        return text, conf

    def publish(self, text: str, conf: float):
        self.pub_text.publish(String(data=text))
        self.pub_conf.publish(Float32(data=float(conf)))
        self.pub_ts.publish(self.get_clock().now().to_msg())
        self.get_logger().info(f'Transcript: "{text}" (conf={conf:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = SpeechToTextNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()



if __name__ == "__main__":
    main()
