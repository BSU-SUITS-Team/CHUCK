import pyautogui

def handle_button(name):
    pyautogui.press(name.lower())

from Buttons import Button
from signal import pause

# Assign buttons to GPIO pins
buttons = {
    "A": Button(17),
    "B": Button(18),
    "C": Button(27),
    "D": Button(22),
    "E": Button(23),
    "F": Button(24)
}

def handle_button(name):
    print(f"Button {name} pressed")

# Attach handlers
for name, button in buttons.items():
    button.when_pressed = lambda n=name: handle_button(n)

pause()

#Handle Scrolling code for buttons
current_page = 0

def scroll_up():
    global current_page
    current_page += 1
    print(f"Page: {current_page}")

def scroll_down():
    global current_page
    current_page -= 1
    print(f"Page: {current_page}")