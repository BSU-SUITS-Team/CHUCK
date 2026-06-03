from display_manager import DisplayManager
from input_handler import InputHandler
from state_manager import StateManager
from Buttons import BUTTON_PINS
from signal import pause
import threading

state = StateManager()
display = DisplayManager()

def handle_input(button_name):
    print(f"{button_name} pressed")

    if button_name == "SCROLL_UP":
        state.scroll_up()
        display.update(state.current_page)

    elif button_name == "SCROLL_DOWN":
        state.scroll_down()
        display.update(state.current_page)

def setup_buttons():
    InputHandler(BUTTON_PINS, handle_input)
    pause()

threading.Thread(target=setup_buttons, daemon=True).start()

display.run()

#cd ~/chuck-main
#python3 main.py
#sudo nano /etc/systemd/system/chuck-main.service
#sudo systemctl daemon-reload
#sudo systemctl enable chuck-main.service
#sudo systemctl start chuck-main.service
#sudo systemctl status chuck-main.service
#journalctl -u chuck-main.service -f
#git init