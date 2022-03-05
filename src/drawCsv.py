from MotorController import MotorController
from threading import Thread

def backgroundLoop():
    controller = MotorController()

    # read data from csv file
    coordinates = []
    csv = open(csvPath, "rt")
    for line in csv:
        delimiterPositions = []
        xyPair = []

        # find delimiter positions in line
        for i in range(0, len(line)):
            if "," == line[i]:
                delimiterPositions.append(i)
        assert(2 == len(delimiterPositions))

        # extract x and y values
        xVal = float(line[0:delimiterPositions[0]])
        yVal = float(line[(delimiterPositions[0] + 1):delimiterPositions[1]])

        # append x and y values to coordinates array
        xyPair.append(xVal)
        xyPair.append(yVal)
        coordinates.append(xyPair)

    while True:
        for i in range(0, len(coordinates)):
            controller.moveXY(coordinates[0], coordinates[1])

if __name__ == "__main__":
    # get name / path of data file
    csvPath = str(input("Please input name / path of CSV file containing coordinates:\n"))
    if csvPath[-4:] != ".csv":
        csvPath += ".csv"

    backgroundThread = Thread(target=backgroundLoop, daemon=True)
    backgroundThread.start()

    input("Press enter to exit.")
