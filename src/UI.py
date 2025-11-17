from vex import *

brain = Brain()

class button:
    def __init__(self, height:int, width:int, posX:int, posY:int, color, text:str) -> None:
        self.height = height
        self.width = width
        self.posX = posX
        self.posY = posY
        self.Pressed = False
        self.color = color
        self.text = text

    def draw(self):
        brain.screen.set_pen_color(self.color)
        brain.screen.draw_rectangle(self.posX, self.posY, self.width, self.height, self.color)
        brain.screen.set_pen_color(Color.BLACK)
        brain.screen.print_at(self.text, self.posX + 10, self.posY + self.height//2 - 5)

    def isPressed(self, touchX:int, touchY:int) -> bool:
        if touchX > self.posX and touchX < self.posX + self.width and touchY > self.posY and touchY < self.posY + self.height:
            self.Pressed = True
        else:
            self.Pressed = False
        return self.Pressed
    
class autonSelector:
    def __init__(self, autons:list, background) -> None:
        self.autons = autons
        self.background = background

    def display(self):
        buttons = []
        brain.screen.draw_image_from_file(self.background, 0, 0)
        for i in range(len(self.autons)):
            buttons.append(button(50, 220, 10, 10 + i*60, Color.BLUE, self.autons[i]))
            buttons[i].draw()
        brain.screen.render()

        while True:
            if brain.screen.pressing():
                touchX = brain.screen.x_position()
                touchY = brain.screen.y_position()
                for i in range(len(buttons)):
                    if buttons[i].isPressed(touchX, touchY):
                        brain.screen.clear_screen()
                        brain.screen.draw_image_from_file(self.background, 0, 0)
                        confirm = button(60, 220, 10, 10, Color.GREEN, "Confirm")
                        cancel = button(60, 220, 250, 10, Color.RED, "Cancel")
                        confirm.draw()
                        cancel.draw()
                        brain.screen.print_at(self.autons[i].__doc__, 10, 80)

                        brain.screen.render()

                        while True:
                            if brain.screen.pressing():
                                touchX = brain.screen.x_position()
                                touchY = brain.screen.y_position()
                                if confirm.isPressed(touchX, touchY):
                                    self.selected = self.autons[i]
                                    return self.selected
                                elif cancel.isPressed(touchX, touchY):
                                    brain.screen.clear_screen()
                                    brain.screen.draw_image_from_file(self.background, 0, 0)
                                    for j in range(len(buttons)):
                                        buttons[j].draw()
                                    brain.screen.render()
                                    break
                            wait(100)
            wait(100)

    


