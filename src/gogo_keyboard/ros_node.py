"""ROS2 node that publishes keyboard events as JSON-encoded std_msgs/String messages.

Run using:
    - python3 -m gogo_keyboard.ros_node
    - python3 -m gogo_keyboard.ros_node -t my_topic
    - python3 -m gogo_keyboard.ros_node --topic my_topic
"""
import argparse
import asyncio
import json
from contextlib import suppress
from dataclasses import asdict

import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

from gogo_keyboard.keyboard import Key, KeySub


def make_ros_msg(key: Key) -> String:
    """Convert a Key object into a JSON-encoded ROS String message."""
    dict_key = asdict(key)
    del dict_key["sdl_event"]
    return String(data=json.dumps(dict_key))


async def async_main(topic: str = "key_press"):
    """Run the async keyboard listener and publish key events to a ROS2 topic.

    Args:
        topic: ROS2 topic to publish key events to
    """
    rclpy.init()
    node = rclpy.create_node("gogo_keyboard")
    pub = node.create_publisher(
        String,
        topic,
        QoSProfile(  # no message lost (hopefuly)
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_ALL,
            durability=DurabilityPolicy.VOLATILE,
        ),
    )
    key_sub = KeySub()
    print(
        f"🦍🦍_keyboard publishing onto `{node.resolve_topic_name(pub.topic)}`. \nListen using `ros2 topic echo {node.resolve_topic_name(pub.topic)}`"
    )
    try:
        async for key in key_sub.listen_reliable():
            msg = make_ros_msg(key)
            pub.publish(msg)
    finally:
        print(f"gogo_keyboard exiting.")
        key_sub.close()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-t",
        "--topic",
        default="key_press",
        help="ROS2 topic to publish key events to",
    )
    args = parser.parse_args()
    with suppress(
        asyncio.CancelledError, KeyboardInterrupt, rclpy._rclpy_pybind11.RCLError
    ):
        asyncio.run(async_main(args.topic))


if __name__ == "__main__":
    main()
