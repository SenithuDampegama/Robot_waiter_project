#!/usr/bin/env python3
import os
import json
import re
import time
from typing import Any, Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

try:
    from openai import OpenAI
except Exception:
    OpenAI = None


DEFAULT_MENU = {
    "items": [
        "chicken burger",
        "beef burger",
        "veggie burger",
        "fries",
        "nuggets",
        "coke",
        "diet coke",
        "sprite",
        "water",
    ],
    "modifiers": [
        "no mayo", "extra mayo",
        "no onions", "extra onions",
        "no cheese", "extra cheese",
        "spicy", "less spicy",
    ],
    "sizes": ["small", "regular", "large"],
}


def extract_json_object(text: str) -> Optional[Dict[str, Any]]:
    """Extract a JSON object from the model output (handles accidental extra text)."""
    if not text:
        return None
    text = text.strip()

    # Try pure JSON first
    try:
        return json.loads(text)
    except Exception:
        pass

    # Try to locate first {...} block
    m = re.search(r"\{.*\}", text, flags=re.DOTALL)
    if not m:
        return None
    blob = m.group(0)
    try:
        return json.loads(blob)
    except Exception:
        return None


def validate_order(order: Dict[str, Any], menu: Dict[str, Any]) -> Tuple[bool, str]:
    """Minimal schema checks + menu constraints."""
    required = ["table_id", "items", "notes", "needs_clarification", "clarification_question", "confidence"]
    for k in required:
        if k not in order:
            return False, f"Missing field: {k}"

    if not isinstance(order["items"], list) or len(order["items"]) == 0:
        return False, "items must be a non-empty list"

    allowed_items = {x.lower() for x in menu.get("items", [])}
    allowed_mods = {x.lower() for x in menu.get("modifiers", [])}
    allowed_sizes = {x.lower() for x in menu.get("sizes", [])}

    for it in order["items"]:
        if not isinstance(it, dict):
            return False, "each item must be an object"

        for kk in ["name", "qty", "mods", "size"]:
            if kk not in it:
                return False, f"item missing field: {kk}"

        name = str(it["name"]).lower().strip()
        if name not in allowed_items:
            return False, f"unknown menu item: {it['name']}"

        qty = it["qty"]
        if not isinstance(qty, int) or qty < 1 or qty > 20:
            return False, f"invalid qty: {qty} for {it['name']}"

        mods = it["mods"]
        if not isinstance(mods, list):
            return False, f"mods must be list for {it['name']}"
        for m in mods:
            mm = str(m).lower().strip()
            if mm and mm not in allowed_mods:
                return False, f"unknown modifier: {m}"

        size = str(it["size"]).lower().strip()
        if size and size not in allowed_sizes:
            return False, f"unknown size: {it['size']}"

    conf = order["confidence"]
    if not isinstance(conf, (int, float)) or conf < 0.0 or conf > 1.0:
        return False, "confidence must be number in [0,1]"

    # table_id can be empty only if clarification needed
    table_id = str(order["table_id"]).strip()
    if table_id == "" and not bool(order["needs_clarification"]):
        return False, "table_id empty but needs_clarification is false"

    return True, ""


class WaiterNLUParser(Node):
    def __init__(self):
        super().__init__("waiter_nlu_parser")

        # parameters
        self.declare_parameter("stt_topic", "/stt/text")
        self.declare_parameter("model", "gpt-5-mini")
        self.declare_parameter("api_key_env", "OPENAI_API_KEY")
        self.declare_parameter("menu_json_path", "")
        # NOTE: kept for backward-compat with your launch file, but we won't send it to GPT-5.
        self.declare_parameter("temperature", 0.1)

        self.stt_topic = self.get_parameter("stt_topic").value
        self.model = self.get_parameter("model").value
        self.api_key_env = self.get_parameter("api_key_env").value
        self.menu_json_path = self.get_parameter("menu_json_path").value
        self.temperature = float(self.get_parameter("temperature").value)

        # load menu
        self.menu = DEFAULT_MENU
        if self.menu_json_path and os.path.exists(self.menu_json_path):
            try:
                with open(self.menu_json_path, "r") as f:
                    self.menu = json.load(f)
                self.get_logger().info(f"Loaded menu from {self.menu_json_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load menu_json_path: {e}. Using DEFAULT_MENU")

        # pubs
        self.pub_order = self.create_publisher(String, "/order/json", 10)
        self.pub_need = self.create_publisher(Bool, "/order/need_clarification", 10)
        self.pub_q = self.create_publisher(String, "/order/clarification_question", 10)
        self.pub_raw = self.create_publisher(String, "/order/raw_llm", 10)

        # sub
        self.sub = self.create_subscription(String, self.stt_topic, self.on_stt, 10)

        # openai client
        self.client = None
        if OpenAI is None:
            self.get_logger().error("Missing openai package. Install: pip3 install openai")
        else:
            api_key = os.getenv(self.api_key_env, "")
            if not api_key:
                self.get_logger().error(f"Missing env var {self.api_key_env}. Export it before running.")
            else:
                self.client = OpenAI(api_key=api_key)

        self.get_logger().info(f"NLU ready. Sub: {self.stt_topic} | Pub: /order/json | model={self.model}")

    def system_prompt(self) -> str:
        return (
            "You are a strict restaurant NLU parser.\n"
            "Convert transcript into EXACTLY ONE JSON object.\n"
            "Rules:\n"
            "- Output ONLY JSON (no markdown, no extra text).\n"
            "- Use ONLY menu items provided.\n"
            "- If anything is missing/unclear, set needs_clarification=true and ask ONE short question.\n"
            "- Do NOT invent items.\n"
            "Schema:\n"
            "{"
            "\"table_id\":\"T3 or empty\","
            "\"items\":[{\"name\":\"<menu item>\",\"qty\":1,\"mods\":[],\"size\":\"small|regular|large\"}],"
            "\"notes\":\"\","
            "\"needs_clarification\":false,"
            "\"clarification_question\":\"\","
            "\"confidence\":0.0"
            "}"
        )

    def build_user_prompt(self, transcript: str) -> str:
        menu_txt = json.dumps(self.menu, indent=2)
        return (
            f"MENU (allowed):\n{menu_txt}\n\n"
            f"TRANSCRIPT:\n{transcript}\n\n"
            "Return ONLY the JSON object."
        )

    def on_stt(self, msg: String):
        transcript = (msg.data or "").strip()
        if not transcript:
            return

        if self.client is None:
            self.get_logger().error("OpenAI client not ready.")
            return

        self.get_logger().info(f"STT: {transcript}")

        t0 = time.time()
        try:
            # IMPORTANT: GPT-5 models reject 'temperature' in Responses API.
            resp = self.client.responses.create(
                model=self.model,
                input=[
                    {"role": "system", "content": self.system_prompt()},
                    {"role": "user", "content": self.build_user_prompt(transcript)},
                ],
            )

            out_text = getattr(resp, "output_text", None)
            if out_text is None:
                out_text = str(resp)

            self.pub_raw.publish(String(data=out_text))

            order = extract_json_object(out_text)
            if order is None:
                return self.publish_clarification("Sorry, I didn’t catch that. Can you repeat your order?")

            ok, reason = validate_order(order, self.menu)
            if not ok:
                self.get_logger().warn(f"Invalid order: {reason}")
                return self.publish_clarification("Sorry — can you confirm the items and quantities?")

            self.pub_order.publish(String(data=json.dumps(order)))
            self.pub_need.publish(Bool(data=bool(order.get("needs_clarification", False))))
            self.pub_q.publish(String(data=str(order.get("clarification_question", ""))))

            self.get_logger().info(f"Order OK in {time.time() - t0:.2f}s")

        except Exception as e:
            self.get_logger().error(f"LLM call failed: {e}")
            self.publish_clarification("Sorry, I had a problem. Please say that again.")

    def publish_clarification(self, question: str):
        self.pub_need.publish(Bool(data=True))
        self.pub_q.publish(String(data=question))


def main():
    rclpy.init()
    node = WaiterNLUParser()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
