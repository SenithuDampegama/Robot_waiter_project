#!/usr/bin/env python3

from enum import Enum
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped


class State(str, Enum):
    IDLE = "IDLE"
    WAITING_FOR_ORDER = "WAITING_FOR_ORDER"
    NAV_TO_KITCHEN = "NAV_TO_KITCHEN"
    WAITING_KITCHEN = "WAITING_KITCHEN"
    NAV_TO_TABLE = "NAV_TO_TABLE"
    DELIVERING = "DELIVERING"
    RETURNING = "RETURNING"
    ERROR = "ERROR"


@dataclass
class Order:
    order_id: str
    table_id: str
    raw: str


class OrchestratorFSM(Node):

    def __init__(self):
        super().__init__("waiter_orchestrator")

        # ---------------- Parameters ----------------
        self.declare_parameter("order_timeout_s", 60.0)
        self.declare_parameter("nav_timeout_s", 45.0)
        self.declare_parameter("kitchen_timeout_s", 60.0)
        self.declare_parameter("deliver_timeout_s", 30.0)
        self.declare_parameter("return_timeout_s", 45.0)
        self.declare_parameter("retry_limit", 2)

        self.declare_parameter("topics.orders", "/orders")
        self.declare_parameter("topics.order_events", "/order_events")
        self.declare_parameter("topics.table_pose", "/table_pose")
        self.declare_parameter("topics.nav_status", "/nav_status")
        self.declare_parameter("topics.kitchen_status", "/kitchen_status")
        self.declare_parameter("topics.mission_state", "/mission_state")
        self.declare_parameter("topics.goal_pose", "/goal_pose")

        self.order_timeout = self.get_parameter("order_timeout_s").value
        self.nav_timeout = self.get_parameter("nav_timeout_s").value
        self.kitchen_timeout = self.get_parameter("kitchen_timeout_s").value
        self.deliver_timeout = self.get_parameter("deliver_timeout_s").value
        self.return_timeout = self.get_parameter("return_timeout_s").value
        self.retry_limit = self.get_parameter("retry_limit").value

        # ---------------- Topics ----------------
        t_orders = self.get_parameter("topics.orders").value
        t_table_pose = self.get_parameter("topics.table_pose").value
        t_nav_status = self.get_parameter("topics.nav_status").value
        t_kitchen_status = self.get_parameter("topics.kitchen_status").value
        t_mission = self.get_parameter("topics.mission_state").value
        t_goal = self.get_parameter("topics.goal_pose").value

        self.mission_pub = self.create_publisher(String, t_mission, 10)
        self.goal_pub = self.create_publisher(PoseStamped, t_goal, 10)

        self.create_subscription(String, t_orders, self.cb_order, 10)
        self.create_subscription(PoseStamped, t_table_pose, self.cb_table_pose, 10)
        self.create_subscription(String, t_nav_status, self.cb_nav_status, 10)
        self.create_subscription(String, t_kitchen_status, self.cb_kitchen_status, 10)

        # ---------------- FSM State ----------------
        self.state = State.WAITING_FOR_ORDER
        self.last_transition_time = self.get_clock().now()
        self.current_order: Optional[Order] = None
        self.table_pose: Optional[PoseStamped] = None

        self.nav_done = False
        self.kitchen_ready = False
        self.retries = 0

        self.timer = self.create_timer(0.1, self.tick)

        self.log_state("System started")

    # ---------------- Callbacks ----------------

    def cb_order(self, msg: String):
        self.current_order = Order(order_id="1", table_id="A", raw=msg.data)
        self.log_state("Order received")

    def cb_table_pose(self, msg: PoseStamped):
        self.table_pose = msg

    def cb_nav_status(self, msg: String):
        if msg.data.upper() == "DONE":
            self.nav_done = True

    def cb_kitchen_status(self, msg: String):
        if msg.data.upper() == "READY":
            self.kitchen_ready = True

    # ---------------- FSM Logic ----------------

    def log_state(self, reason=""):
        self.get_logger().info(f"[FSM] {self.state.value} {reason}")
        self.mission_pub.publish(String(data=self.state.value))

    def transition(self, new_state: State, reason=""):
        self.state = new_state
        self.last_transition_time = self.get_clock().now()
        self.nav_done = False
        self.kitchen_ready = False
        self.log_state(reason)

    def elapsed(self):
        return (self.get_clock().now() - self.last_transition_time).nanoseconds / 1e9

    def tick(self):

        if self.state == State.WAITING_FOR_ORDER:
            if self.current_order:
                self.transition(State.NAV_TO_KITCHEN, "Going to kitchen")

        elif self.state == State.NAV_TO_KITCHEN:
            self.transition(State.WAITING_KITCHEN, "Waiting in kitchen")

        elif self.state == State.WAITING_KITCHEN:
            if self.kitchen_ready or self.elapsed() > self.kitchen_timeout:
                self.transition(State.NAV_TO_TABLE, "Order ready")

        elif self.state == State.NAV_TO_TABLE:
            if self.table_pose:
                self.goal_pub.publish(self.table_pose)
                self.transition(State.DELIVERING, "Arrived at table")

        elif self.state == State.DELIVERING:
            if self.elapsed() > self.deliver_timeout:
                self.transition(State.RETURNING, "Delivery done")

        elif self.state == State.RETURNING:
            if self.elapsed() > self.return_timeout:
                self.current_order = None
                self.transition(State.WAITING_FOR_ORDER, "Returned home")

        elif self.state == State.ERROR:
            self.transition(State.WAITING_FOR_ORDER, "Recovered")


def main(args=None):
    rclpy.init(args=args)
    node = OrchestratorFSM()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
