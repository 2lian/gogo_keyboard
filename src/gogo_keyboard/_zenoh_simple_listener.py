from contextlib import suppress
import os
import time
from typing import Optional

import zenoh


def main(topic: str = "key_press", config_path: Optional[str] = None):
    if config_path is None:
        conf = zenoh.Config()
    else:
        filepath = os.path.expanduser(config_path)
        conf = zenoh.Config.from_file(filepath)

    with zenoh.open(conf) as session:
        print(f"Subscribing to '{topic}'")
        def listener(sample: zenoh.Sample):
            print(
                f"Received {sample.kind} ('{sample.key_expr}': '{sample.payload.to_string()}')"
            )
        session.declare_subscriber(topic, listener)
        while True:
            time.sleep(1)


# --- Command line argument parsing --- --- --- --- --- ---
if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(prog="z_sub", description="zenoh sub example")
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

    with suppress(KeyboardInterrupt):
        main(args.topic, args.config)
