import pyautogui

def handle_button(name):
    pyautogui.press(name.lower())


from ButtonsDraft1 import Button
from signal import pause
#remeber to ask what pins the buttons have

buttonA = Button(17),
buttonB = Button(17),
buttonC = Button(17),
buttonD = Button(17),
buttonE = Button(17),
buttonF = Button(17)

def button_pressed():
    print("Numeric Data Button was pressed")

def button_pressed1():
    print("Data Button was pressed")

def button_pressed2():
    print("Warning/Caution and Errors Button was pressed")

def button_pressed3():
    print(" Scroll UP Button was pressed")

def button_pressed4():
    print(" Scroll Down Button was pressed")
    

def button_pressed5():
    print(" Audio notes Button was pressed")

buttonA.when_pressed = button_pressed
buttonB.when_pressed = button_pressed1
buttonC.when_pressed = button_pressed2
buttonD.when_pressed = button_pressed3
buttonE.when_pressed = button_pressed4
buttonF.when_pressed = button_pressed5
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