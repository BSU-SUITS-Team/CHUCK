from Buttons import Button

class InputHandler:
    def __init__(self, config, callback):
        self.buttons = {}
        self.callback = callback

        for name, pin in config.items():
            btn = Button(pin)
            btn.when_pressed = lambda n=name: self.callback(n)
            self.buttons[name] = btn