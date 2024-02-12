import websocket
import rel
import json
import argparse


def filter_on(event):
    def on_message(ws, message):
        json_message = json.loads(message)
        if json_message["type"] == event or event==None:
            print(json_message["data"])
    return on_message


def on_error(ws, error):
    print(error)


def on_close(ws, close_status_code, close_msg):
    # ws.close()
    print("### closed ###")


def on_open(ws):
    print("Opened connection")


if __name__ == "__main__":
    # Initialize parser
    parser = argparse.ArgumentParser(description="Websocket client for groundcontrol-api")
    parser.add_argument("-e", "--Event", help = "Event string to filter on", type = str, default = None)
    args = parser.parse_args()
    event_to_filter_in = args.Event
    websocket.enableTrace(False)
    filtered_on_message = filter_on(event_to_filter_in)
    ws = websocket.WebSocketApp(
        "ws://groundcontrol-api:8181/ws/events",
        on_open=on_open,
        on_message=filtered_on_message,
        on_error=on_error,
        on_close=on_close,
    )

    ws.run_forever(
        dispatcher=rel, reconnect=5
    )  # Set dispatcher to automatic reconnection, 5 second reconnect delay if connection closed unexpectedly
    rel.signal(2, rel.abort)  # Keyboard Interrupt
    rel.dispatch()
