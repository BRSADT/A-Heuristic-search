import utils
from Node import Node
from Position import Position
from utils import manhattan_distance as manhattan


class A_Star_Algorithm():

    def __init__(self):
        """self.start = [
            #     0    1   2   3
            ["M1", "#", "*", "M3"],  # 0
            ["*", "#", "*", "*"],  # 1
            ["M2", "*", "R", "*"],  # 2
            ["*", "*", "*", "*"]  # 3
        ]
        self.goal = [
            ["", "#", "", ""],
            ["", "#", "", ""],
            ["", "", "", ""],
            ["", "M3", "M2", "M1"]
        ]"""
        self.start = [
           #     0    1   2   3
           ["M2", "*", "", "M3"],  # 0
           ["#", "#", "#", "*"],  # 1
           ["*", "#", "*", "*"],  # 2
           ["*", "R", "*", "*"]  # 3
        ]
        self.goal = [
           ["*", "*", "*", "*"],  # 0
           ["#", "#", "#", "*"],  # 1
           ["*", "#", "*", "*"],  # 2
           ["*", "M3", "M2", ""]
        ]
        self.track = []
        self.openList = []
        self.closedList = []
        self.mapObjetives = {}
        self.currentPosition = Position(0, 0)
        self.origin = Position(0, 0)
        self.X = 0  # axe X
        self.Y = 0  # axe Y
        self.mapDistancesShelfsToTheirObjective = {}
        self.currentMap = []
        self.state="SearchShelf"
        self.iteration=0

    def set_state(self, state):
        self.state = state

    def get_state(self):
        return self.state

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
        if leftX < self.X and leftX >= 0:
            adjacentPositions.append(Position(leftX, leftY))
        if upY < self.Y and upY >= 0:
            adjacentPositions.append(Position(upX, upY))
        if rightX < self.X and rightX >= 0:
            adjacentPositions.append(Position(rightX, rightY))
        if downY < self.Y and downY >= 0:
            adjacentPositions.append(Position(downX, downY))

        return adjacentPositions



##use this and instead of get the less cost, get a list
    def SearchObjectiveStateGetNextNode(self, node,objective):
        ListOfNodes=[]
        listAdjNodes = self.getAdjacentPositions(node.getX(), node.getY())
        minorCost = 10000000
        nodeToMove = ""
        for adjNode in listAdjNodes:
            #cost = self.calculateDistanceToAnObjective(adjNode,objective)
            cost = self.calculateCostToObjective(adjNode,objective)
            self.currentMap[adjNode.getX()][adjNode.getY()].setCostToGoal(cost)
            ListOfNodes.append(self.currentMap[adjNode.getX()][adjNode.getY()])

        return ListOfNodes

    def SearchShelfNodeWithLessCostAllAdjacent(self,node):
        listAdjNodes= self.getAdjacentPositions(node.getX(),node.getY())
        minorCost= 10000000
        nodeToMove = ""
        for adjNode in listAdjNodes:
            nodeToMoveCal,minorCostAdjNode = self.SearchShelfNodeWithLessCost(adjNode,minorCost)
            if minorCostAdjNode < minorCost:
                minorCost = minorCostAdjNode
                nodeToMove = nodeToMoveCal
        return self.mapObjetives[nodeToMove]

    def SearchShelfNodeWithLessCost(self,position,minorCost):
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


    def setScenario(self):
        print("")
        self.createListObjetives()
        self.setGoalsOfShelf()
        self.mapDistancesShelfsToTheirObjective = self.calculateDistanceShelfToItsObjective()

    def startAlgorithm(self):
        self.set_state("SearchShelf")
        currentNode= self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()]
        pos = currentNode.getPosition()
        ShelfToTake = self.SearchShelfNodeWithLessCost(pos,10000)
        shelfNode= self.mapObjetives[ShelfToTake[0]]
        currentNode.setCostToGoal(self.calculateDistanceToAnObjective(currentNode.getPosition(), shelfNode.getPosition()))
        self.openList.append(self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()])
        self.track.append(currentNode)
        self.A_star_algorithm_SearchShelf(currentNode)

    def stopAlgorithm(self):
        print("stop")
    def A_star_algorithm_SearchShelf(self, currentNode):
        self.currentPosition = currentNode.getPosition()

        """for node in self.track:
            print(f"Node : {node.getPosition().getX()}, {node.getPosition().getY()}")"""

        if  not self.mapObjetives:
            print("End of algorithm!!!!")
        if currentNode.type == "Shelf" and currentNode.onObjective==False :
            print("found shelf activate search objective-----------")
            self.set_state("MoveShelf")
            # clean open list
            self.openList = []
            #add first
            currentNode.setCostToGoal( self.calculateDistanceToAnObjective(currentNode.getPosition(), self.mapObjetives[currentNode.name].getGoalPosition()))
            self.openList.append(self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()])
            # clean close list except for blocked , lets keep those
            self.closedList = []
            print(f"shelf to go {self.mapObjetives[currentNode.name].name} goal {self.mapObjetives[currentNode.name].getGoalPosition().getX()}, {self.mapObjetives[currentNode.name].getGoalPosition().getY()} ")
            self.updateMapRobot(currentNode)
            self.printCurrentMapObjective(currentNode)
            print("\n\nstart A* for moving the shelf \n\n")
            self.A_star_algorithm_MoveShelf(currentNode,self.mapObjetives[currentNode.name])


        elif currentNode.type == "Obstacle":
            print(f"found obstacle on {currentNode.getPosition().getX()}, {currentNode.getPosition().getY()} try another node,go back")
            self.closedList=utils.addNodeIfNotInList({currentNode},self.closedList)
            nodeToGo = self.track[-1]
            self.track.pop()
            print(f"go back to {nodeToGo.getPosition().getX()}, {nodeToGo.getPosition().getY()} try another node,go back")
            self.openList = utils.removeElementInList(currentNode, self.openList)
            self.printCurrentMapBloqued(currentNode)
            self.A_star_algorithm_SearchShelf(nodeToGo)
        else:
            ShelfToTake=self.SearchShelfNodeWithLessCostAllAdjacent(currentNode.getPosition())
            Adjnodes = self.SearchObjectiveStateGetNextNode(currentNode.getPosition(), ShelfToTake.getPosition())
            #get nodes only that are not in closed list
            nodes = utils.removeNodeIfInList(Adjnodes,self.closedList)
            #add currentNode to close list
            self.closedList = utils.addNodeIfNotInList({currentNode}, self.closedList)

            nodes= sorted(nodes, key=lambda x: x.costToGoal)
            #remove current node from open
            self.openList=utils.removeElementInList(currentNode,self.openList)
            #add
            self.openList=utils.updateNodeIfInList(nodes,self.openList)

            #if open list is empty end algorithm
            if not  self.openList:
                print("End of algorithm")
            else:

                #probably we need to add a check
                nodeToGo=  self.openList[0]

                self.updateMapRobot(currentNode)
                self.printCurrentMap()

                self.printLists()

                #check if node to move is in adj node if not go back
                if not utils.isNodeInList(nodeToGo,Adjnodes):
                    nodeToGo = self.track[-1]
                    self.track.pop()
                    print(f"go back to node-> \t\t{nodeToGo.getPosition().getX()}, {nodeToGo.getPosition().getY()}")
                    self.A_star_algorithm_SearchShelf(nodeToGo)
                else:

                    print(f"move to node-> \t\t{nodeToGo.getPosition().getX()}, {nodeToGo.getPosition().getY()}")

                    self.track.append(currentNode)
                    self.A_star_algorithm_SearchShelf(nodeToGo)

    def printLists(self):
        print(f"                 \tx, y                              ")
        for node in self.openList:
            print(
                f"OPEN LIST Node : \t{node.getPosition().getX()}, {node.getPosition().getY()} cost: {node.costToGoal}")
        for node in self.closedList:
            print(
                f"CLOSED LIST Node : \t{node.getPosition().getX()}, {node.getPosition().getY()} cost: {node.costToGoal}")

    def updateMapRobot(self,nodeToGo):
        for i in range(self.X):
            for j in range(self.Y):
                if self.currentMap[i][j].type == "Robot":
                    self.currentMap[i][j].name = "*"
                    self.currentMap[i][j].type = ""
                if self.currentMap[i][j].getPosition().getX() == nodeToGo.getPosition().getX() and self.currentMap[i][j].getPosition().getY() == nodeToGo.getPosition().getY():
                    if self.currentMap[i][j].type != "Shelf":
                        self.currentMap[i][j].name = "R"
                        self.currentMap[i][j].type = "Robot"

    def updateMapRobotShelf(self,nodeToGo,shelf):
        for i in range(self.X):
            for j in range(self.Y):
                if self.currentMap[i][j].name == shelf.name:
                    self.currentMap[i][j].name = "*"
                    self.currentMap[i][j].type = ""
                if self.currentMap[i][j].getPosition().getX() == nodeToGo.getPosition().getX() and self.currentMap[i][j].getPosition().getY() == nodeToGo.getPosition().getY():
                    self.currentMap[i][j].name = shelf.name
                    self.currentMap[i][j].type = shelf.type


    def printCurrentMap(self):
        print("\ty0\ty1\ty2\ty3")
        for i in range(self.X):
            print(f"x{i}", end="\t")
            for j in range(self.Y):
                print(f'{self.currentMap[i][j].name}', end="\t")
            print("")

    def printCurrentMapBloqued(self,node):
        print("\ty0\ty1\ty2\ty3")
        for i in range(self.X):
            print(f"x{i}", end="\t")
            for j in range(self.Y):


                if node.getPosition().getX() == i and node.getPosition().getY() == j:
                    print(f'X', end="\t")
                else:
                    if self.currentMap[i][j].type == "Robot":
                        print(f'*', end="\t")
                    else:
                        print(f'{self.currentMap[i][j].name}', end="\t")
            print("")

    def printCurrentMapObjective(self,node):
        print("\ty0\ty1\ty2\ty3")
        for i in range(self.X):
            print(f"x{i}", end="\t")
            for j in range(self.Y):
                if node.getPosition().getX() == i and node.getPosition().getY() == j:
                    print(f'[{self.currentMap[i][j].name}]', end="\t")
                else:
                    print(f'{self.currentMap[i][j].name}', end="\t")
            print("")

    def A_star_algorithm_MoveShelf(self, currentNode,Shelf):
            self.currentPosition = currentNode.getPosition()
            #self.track.append(currentNode)
            """for node in self.track:
                print(f"TRACK Node : {node.getPosition().getX()}, {node.getPosition().getY()}")"""


            if (currentNode.name != Shelf.name )  and (currentNode.type == "Shelf" or currentNode.type == "Obstacle" )  :
                print(
                    f"found obstacle on {currentNode.getPosition().getX()}, {currentNode.getPosition().getY()} try another node,go back")
                self.closedList = utils.addNodeIfNotInList({currentNode}, self.closedList)
                # self.openList.pop()

                nodeToGo = self.track[-1]
                self.track.pop()

                self.openList = utils.removeElementInList(currentNode, self.openList)
                self.printCurrentMapBloqued(currentNode)
                self.A_star_algorithm_MoveShelf(nodeToGo,Shelf)


            elif currentNode.getPosition().getX() == Shelf.getGoalPosition().getX() and currentNode.getPosition().getY() == Shelf.getGoalPosition().getY() :
                self.updateMapRobotShelf(currentNode, Shelf)
                self.printCurrentMap()
                print(f"found goal for shelf {Shelf.name} ***" )
                currentNode.onObjective = True
                self.mapObjetives.pop(Shelf.name)

                if not self.mapObjetives:
                    print("End of algorithm!!!!")
                else:
                    self.openList = []
                    self.closedList = []

                    self.set_state("SearchShelf")
                    currentNode = self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()]
                    pos = currentNode.getPosition()
                    ShelfToTake = self.SearchShelfNodeWithLessCost(pos, 10000)
                    shelfNode = self.mapObjetives[ShelfToTake[0]]
                    currentNode.setCostToGoal(
                        self.calculateDistanceToAnObjective(currentNode.getPosition(), shelfNode.getPosition()))
                    self.openList.append(self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()])
                    self.track.append(currentNode)
                    self.updateMapRobot(currentNode)
                    self.printCurrentMapObjective(currentNode)
                    print("\t\t start A* to search a Shelf\t\t")
                    self.A_star_algorithm_SearchShelf(currentNode)



            else:
                Adjnodes = self.SearchObjectiveStateGetNextNode(currentNode.getPosition(), Shelf.getGoalPosition())
                # get nodes only that are not in closed list
                nodes = utils.removeNodeIfInList(Adjnodes, self.closedList)
                # add currentNode to close list
                self.closedList = utils.addNodeIfNotInList({currentNode}, self.closedList)
                # remove current node from open
                self.openList = utils.removeElementInList(currentNode, self.openList)
                # add

                nodes = sorted(nodes, key=lambda x: x.costToGoal)

                self.openList = utils.updateNodeIfInList(nodes, self.openList)

                # if open list is empty end algorithm
                if not self.openList:
                    print("End of algorithm")
                else:



                    # probably we need to add a check
                    nodeToGo = self.openList[0]
                    self.printLists()
                    # check if node to move is in adj node if not go back
                    if not utils.isNodeInList(nodeToGo, Adjnodes):
                        nodeToGo = self.track[-1]
                        self.track.pop()
                        print(f"go back to node-> {nodeToGo.getPosition().getX()}, {nodeToGo.getPosition().getY()}")
                        self.updateMapRobotShelf(currentNode,Shelf)
                        self.printCurrentMap()
                        self.A_star_algorithm_MoveShelf(nodeToGo,Shelf)
                    else:
                        self.updateMapRobotShelf(currentNode,Shelf)
                        self.printCurrentMap()
                        print(f"move to node-> {nodeToGo.getPosition().getX()}, {nodeToGo.getPosition().getY()}")

                        self.track.append(currentNode)
                        self.A_star_algorithm_MoveShelf(nodeToGo,Shelf)
