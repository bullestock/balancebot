from gpiozero import Button

class ResetButton:
    def __init__(self):
        self.button = Button(27)
        
    def is_pressed(self):
        return self.button.is_pressed
