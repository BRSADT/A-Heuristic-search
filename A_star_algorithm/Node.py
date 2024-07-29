from Position import Position
class Node:
    def __init__(self, name,positionX,positionY,type):
        self.name = name
        self.currentPosition=  Position(positionX,positionY)
        self.goalPosition = Position(0,0)
        self.type=type
        self.costToGoal=0
        self.onObjective=False
    def getName(self):
        return self.name
    def setGoalPosition(self, goalPositionX,goalPositionY):
        self.goalPosition.setX(goalPositionX)
        self.goalPosition.setY(goalPositionY)

    def getGoalPosition(self):
        return self.goalPosition

    def setCostToGoal(self, costToGoal):
        self.costToGoal = costToGoal

    def getCostToGoal(self):
        return self.costToGoal

    def getPosition(self):
        return self.currentPosition

    def setPosition(self,pos):
        self.currentPosition = pos