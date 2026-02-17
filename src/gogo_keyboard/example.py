import asyncio
import json
from contextlib import suppress
from dataclasses import asdict

import sdl2
import sdl2.ext

from . import codes
from .keyboard import Key, KeySub


async def detect_window_closed(event: asyncio.Event):
    await event.wait()
    print("Gorilla window closed by user.")


async def detect_CtrlC(key_sub: KeySub):
    async for k in key_sub.listen_reliable(exit_on_close=True):
        if k.symbol == "C" and (k.modifiers & codes.MODIFIER_LCTRL) and k.is_pressed:
            print("Ctrl+C in Gorilla window by user.")
            return


async def print_keys(key_sub: KeySub):
    async for k in key_sub.listen_reliable(exit_on_close=True):
        print(json.dumps({k: str(v) for k, v in asdict(k).items()}, indent=4))


async def async_main():
    window_closed_event = asyncio.Event()
    key_sub = KeySub(
        termination_callback=window_closed_event.set,
    )

    try:
        print_task = asyncio.ensure_future(print_keys(key_sub))
        close_task = asyncio.ensure_future(detect_window_closed(window_closed_event))
        ctrl_task = asyncio.ensure_future(detect_CtrlC(key_sub))
        await asyncio.wait(
            [print_task, close_task, ctrl_task], return_when=asyncio.FIRST_COMPLETED
        )
        print_task.cancel()
        close_task.cancel()
        ctrl_task.cancel()
        await asyncio.wait([print_task, close_task, ctrl_task])
    finally:
        key_sub.close()


def main():
    try:
        asyncio.run(async_main())
    except KeyboardInterrupt:
        print("KeyboardInterrupt in python process")
    finally:
        sdl2.ext.quit()
    print("Exited cleanly :)")


if __name__ == "__main__":
    main()
