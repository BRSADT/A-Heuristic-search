from Node import Node
from Position import Position
from utils import manhattan_distance as manhattan


class A_Star_Algorithm():

    def __init__(self):
        self.start = [
            #     0    1   2   3
            ["M1", "#", "", "M3"],  # 0
            ["", "#", "", ""],  # 1
            ["M2", "", "R", ""],  # 2
            ["", "", "", ""]  # 3
        ]
        self.goal = [
            ["", "#", "", ""],
            ["", "#", "", ""],
            ["", "", "", ""],
            ["", "M3", "M2", "M1"]
        ]
        self.mapObjetives = {}
        self.currentPosition = Position(0, 0)
        self.origin = Position(0, 0)
        self.X = 0  # axe X
        self.Y = 0  # axe Y
        self.mapDistancesShelfsToTheirObjective = {}
        self.currentMap = []

    def set_start(self, s):
        self.start = s

    def set_goal(self, g):
        self.goal = g

    def createListObjetives(self):

        # get x and y of self.start. We are assumming non empty matrix and cuadratical
        self.X = len(self.start)
        self.Y = len(self.start[0])

        self.currentMap = [[0 for j in range(self.Y)] for i in range(self.X)]

        for i in range(self.X):
            for j in range(self.Y):
                if self.start[i][j].startswith("M"):
                    # it is a shelf lets add it to list of objective and add its start
                    self.mapObjetives[self.start[i][j]] = Node(self.start[i][j], i, j, "Shelf")
                    # add node to current map
                    self.currentMap[i].insert(j, Node(self.start[i][j], i, j, "Shelf"))
                elif self.start[i][j].startswith("R"):
                    self.currentPosition = Position(i,j)
                    self.origin = Position(i,j)
                    self.currentMap[i].insert(j, Node(self.start[i][j], i, j, "Robot"))
                elif self.start[i][j].startswith("#"):
                    self.currentMap[i].insert(j, Node(self.start[i][j], i, j, "Obstacle"))
                else:
                    self.currentMap[i].insert(j, Node(self.start[i][j], i, j, ""))

    def setGoalsOfShelf(self):
        # get x and y of self.start. We are assumming non empty matrix and cuadratical
        for i in range(self.X):
            for j in range(self.Y):
                if self.goal[i][j].startswith("M"):
                    # it is a shelf lets add it to list of objective and add its start
                    self.mapObjetives[self.goal[i][j]].setGoalPosition(i, j)

    def getAdjacentPositions(self, currentPositionX, currentPositionY):
        adjacentPositions = []
        rightX = currentPositionX + 1
        rightY = currentPositionY

        upX = currentPositionX
        upY = currentPositionY + 1

        leftX = currentPositionX - 1
        leftY = currentPositionY

        downX = currentPositionX
        downY = currentPositionY - 1
        if leftX < self.X and leftX > 0:
            adjacentPositions.append(Position(leftX, leftY))
        if upY < self.Y and upY > 0:
            adjacentPositions.append(Position(upX, upY))
        if rightX < self.X and rightX > 0:
            adjacentPositions.append(Position(rightX, rightY))
        if downY < self.Y and downY > 0:
            adjacentPositions.append(Position(downX, downY))

        return adjacentPositions

    def setScenario(self):
        self.createListObjetives()
        self.setGoalsOfShelf()
        self.mapDistancesShelfsToTheirObjective = self.calculateDistanceShelfToItsObjective()
        print()

#TO TEST
    def SearchObjectiveStateGetNextNode(self, node,objective):
        listAdjNodes = self.getAdjacentPositions(node.getX(), node.getY())
        minorCost = 10000000
        nodeToMove = ""
        for adjNode in listAdjNodes:
            if  self.calculateDistanceToAnObjective(adjNode,objective) < minorCost :
                nodeToMove = self.currentMap[adjNode.getX()][adjNode.getY()]
        return nodeToMove

    def SearchShelfStateGetNextNode(self,node):
        listAdjNodes= self.getAdjacentPositions(node.getX(),node.getY())
        minorCost= 10000000
        nodeToMove = ""
        for adjNode in listAdjNodes:
           nodeToMove,minorCost = self.SearchShelfStategetNodeWithLessCost(adjNode,minorCost)
        return nodeToMove

    def SearchShelfStategetNodeWithLessCost(self,position,minorCost):
        mapCosts= self.calculateCostToShelfs(position)
        nodeToMove = ""
        for key, value in mapCosts.items():
            if value < minorCost:
                nodeToMove=key
                minorCost = value
        return (nodeToMove,minorCost)


    def calculateCostToShelfs(self,pos):
        mapCosts = {}
        mapDistancesNodeToShelfs = self.calculateDistanceToAllShelfs(pos)
        costOriginToNode= self.calculateDistanceToAnObjective(self.origin,pos)

        for value in self.mapObjetives.values():
            mapCosts[value.getName()] = costOriginToNode + mapDistancesNodeToShelfs[value.getName()] +  self.mapDistancesShelfsToTheirObjective[value.getName()]
        return mapCosts

    def calculateCostToObjective(self,pos1, pos2):
        costOriginToNode= self.calculateDistanceToAnObjective(self.origin,pos1)
        costNodeToNode= self.calculateDistanceToAnObjective(pos1,pos2)
        return costOriginToNode + costNodeToNode


    def calculateDistanceShelfToItsObjective(self):
        mapDistance = {}
        for value in self.mapObjetives.values():
            mapDistance[value.getName()] = manhattan( value.currentPosition, value.goalPosition)
        return mapDistance
    def calculateDistanceToAllShelfs(self, pos):
        mapDistance = {}
        for value in self.mapObjetives.values():
            mapDistance[value.getName()] = manhattan(pos, value.currentPosition)
        return mapDistance

    def calculateDistanceToAnObjective(self, Position1, Position2):
        return manhattan(Position1, Position2)
