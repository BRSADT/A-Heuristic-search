from Position import Position
class Node:
    def __init__(self, name,positionX,positionY,type):
        self.name = name
        self.currentPosition=  Position(positionX,positionY)
        self.goalPosition = Position(0,0)
        self.type=type

    def getName(self):
        return self.name
    def setGoalPosition(self, goalPositionX,goalPositionY):
        self.goalPosition.setX(goalPositionX)
        self.goalPosition.setY(goalPositionY)

    def getGoalPosition(self):
        return self.goalPosition

