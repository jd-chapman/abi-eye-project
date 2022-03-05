from MotorController import MotorController
from threading import Thread
from time import sleep

def backgroundLoop():
    controller = MotorController()

    while True:
        for i in range(0, 101):
            xVal = i
            yVal = 0
            controller.moveXY(xVal, yVal)
        for i in range(0, 101):
            xVal = 100
            yVal = i
            controller.moveXY(xVal, yVal)
        for i in range(100, -1, -1):
            xVal = i
            yVal = 100
            controller.moveXY(xVal, yVal)
        for i in range(100, -1, -1):
            xVal = 0
            yVal = i
            controller.moveXY(xVal, yVal)

if __name__ == "__main__":
    backgroundThread = Thread(target=backgroundLoop, daemon=True)
    backgroundThread.start()

    input("Press enter to exit.")
