import json
import threading
import time

import socketio


sio = socketio.Client()
result_event = threading.Event()
latest_result = None


@sio.on("skill_result")
def on_skill_result(data):
    global latest_result
    latest_result = data
    result_event.set()


def run_skill(name, parameters=None, timeout=15):
    global latest_result
    latest_result = None
    result_event.clear()
    sio.emit(
        "execute_skill",
        {
            "robot_id": "UAV_1",
            "skill_name": name,
            "parameters": parameters or {},
        },
    )
    if not result_event.wait(timeout):
        raise TimeoutError(f"Timed out waiting for {name}")
    if not latest_result.get("ok"):
        raise RuntimeError(f"{name} failed: {latest_result}")
    print(json.dumps(latest_result, ensure_ascii=False))
    return latest_result


sio.connect("http://127.0.0.1:5001", transports=["polling"])
try:
    run_skill("takeoff", {"altitude": 5.0})
    run_skill("fly_relative", {"forward": 10.0, "right": 2.0, "up": 1.0, "speed": 15.0})
    run_skill("get_position")
    run_skill("land")
finally:
    sio.disconnect()

