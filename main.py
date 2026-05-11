from signal import pause
from Buttons import BUTTON_PINS
from input_handler import InputHandler
from state_manager import StateManager
from display_manager import DisplayManager
from communication import send_to_hololens

state = StateManager()
display = DisplayManager()



def handle_input(button_name):
    if button_name == "SCROLL_UP":
        state.scroll_up()
        display.update(state.current_page)

    elif button_name == "SCROLL_DOWN":
        state.scroll_down()
        display.update(state.current_page)

    else:
        send_to_hololens(button_name)
        print(f"Sent {button_name} to HoloLens")

InputHandler(BUTTON_PINS, handle_input)

pause()

#Display
from display_manager import DisplayManager

display = DisplayManager()

def handle_input(button_name):
    if button_name == "SCROLL_UP":
        state.scroll_up()
        display.update(state.current_page)

    elif button_name == "SCROLL_DOWN":
        state.scroll_down()
        display.update(state.current_page)

display.run()

