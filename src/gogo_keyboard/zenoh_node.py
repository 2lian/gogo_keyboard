"""Zenoh publisher that publishes keyboard events as JSON-encoded messages.

Run using:
    - python3 -m gogo_keyboard.zenoh_node
"""

import argparse
import asyncio
import json
import os
from contextlib import suppress
from dataclasses import asdict
from typing import Optional

import zenoh

from gogo_keyboard.keyboard import Key, KeySub


def make_msg(key: Key) -> str:
    """Convert a Key object into a JSON-encoded message."""
    dict_key = asdict(key)
    del dict_key["sdl_event"]
    return json.dumps(dict_key)


async def async_main(topic: str = "key_press", config_path: Optional[str] = None):
    """Run the async keyboard listener and publish key events to a zenoh key_expr.

    Args:
        topic: key_expr to publish key events to
    """
    if config_path is None:
        config = zenoh.Config()
    else:
        filepath = os.path.expanduser(config_path)
        config = zenoh.Config.from_file(filepath)
    with zenoh.open(config) as ses:
        pub = ses.declare_publisher(topic, reliability=zenoh.Reliability.RELIABLE)
        key_sub = KeySub()
        print(
            f"🦍🦍_keyboard publishing onto `{pub.key_expr}`. \nListen using `python3 -m gogo_keyboard._zenoh_simple_listener`"
        )
        try:
            async for key in key_sub.listen_reliable(exit_on_close=True):
                msg = make_msg(key)
                pub.put(msg)
        finally:
            print(f"gogo_keyboard exiting.")
            key_sub.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-c",
        "--config",
        dest="config",
        default=None,
        help="Zenoh config path",
    )
    parser.add_argument(
        "-t",
        "--topic",
        dest="topic",
        default="key_press",
        help="ROS2 topic to publish key events to",
    )
    args = parser.parse_args()
    with suppress(asyncio.CancelledError, KeyboardInterrupt):
        asyncio.run(async_main(args.topic, args.config))


if __name__ == "__main__":
    main()
