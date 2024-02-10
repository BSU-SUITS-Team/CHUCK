import websocket
import _thread
import time
import rel
import json


def on_message(ws, message):
    json_message = json.loads(message)
    print(json_message.keys())
    if json_message["type"] == "procedure":
        print(json_message["data"])


def on_error(ws, error):
    print(error)


def on_close(ws, close_status_code, close_msg):
    # ws.close()
    print("### closed ###")


def on_open(ws):
    print("Opened connection")


if __name__ == "__main__":
    websocket.enableTrace(False)
    ws = websocket.WebSocketApp(
        "ws://groundcontrol-api:8181/ws/events",
        on_open=on_open,
        on_message=on_message,
        on_error=on_error,
        on_close=on_close,
    )

    ws.run_forever(
        dispatcher=rel, reconnect=5
    )  # Set dispatcher to automatic reconnection, 5 second reconnect delay if connection closed unexpectedly
    rel.signal(2, rel.abort)  # Keyboard Interrupt
    rel.dispatch()
