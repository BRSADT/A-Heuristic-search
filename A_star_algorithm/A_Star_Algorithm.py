import utils
from Node import Node
from Position import Position
from utils import manhattan_distance as manhattan


class A_Star_Algorithm():

    """es la inicializacion de nuestras variables donde:
    self.start contiene el mapa original, y self.goal contiene el
    mapa de salida, o sea, como queremos que se muevan nuestros almacenes
    self.track nos ayudara a registrar como se va moviendo nuestro robot, los
    nodos por los que pasa, asi , si tenemos que retroceder por algun bloqueo,
    podra irse al estado anterior. openlist y closedList son las listas que nos
    ayudaran a ir guardando los nodos segun el caso, en openlist se guardan los nodos
    que son potenciales a visitar y en closed los nodos que ya visitamos.
    currentPosition va guardando la posicion del robot, y origin se actualizara
    cada que se corra el algoritmo para A*. CurrentMap guardara el mapa en cada iteracion
    mapDistancesShelfsToTheirObjective guardara la distancia de cada almacen a su objetivo"""
    def __init__(self):
        self.start = [
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
    def setScenario(self):
        print("")
        self.createListObjetives()
        self.setGoalsOfShelf()
        self.mapDistancesShelfsToTheirObjective = self.calculateDistanceShelfToItsObjective()




    """esta funcion recorrera el mapa start, para que currentMap pueda ser poblado, ya no con strings sino con una 
    estructura llamada Node, tambien poblara el mapa de Objetivos, los cuales detectaran si es un almacen y guardaran
    el nombre del almacen y su nodo, el cual contiene datos como nombre, tipo, posicion inicial, posicion final, y costo. 
    """
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

    """este mapa recorrera el mapa de goal, para asi definir el objetivo de los almacenes y actualizarlo en el mapObjectives"""
    def setGoalsOfShelf(self):
        # get x and y of self.start. We are assumming non empty matrix and cuadratical
        for i in range(self.X):
            for j in range(self.Y):
                if self.goal[i][j].startswith("M"):
                    # it is a shelf lets add it to list of objective and add its start
                    self.mapObjetives[self.goal[i][j]].setGoalPosition(i, j)

    """esta funcion calcula la distancia de cada almacen a su objetivo planteado"""
    def calculateDistanceShelfToItsObjective(self):
        mapDistance = {}
        for value in self.mapObjetives.values():
            mapDistance[value.getName()] = manhattan( value.currentPosition, value.goalPosition)
        return mapDistance


    """esta funcion calculara los nodos adyacentes cada nodo, solo los que esten dentro del mapa
    ya que tiene la condicion de detectar los nodos adyacentes fuera del mapa"""
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



    """regresara una lista de nodos adyacentes del nodo actual, pero actualizando el parametro
    costToGoal de cada nodo adyacente al nodo."""
    def SearchObjectiveStateGetNextNode(self, node,objective):
        ListOfNodes=[]
        listAdjNodes = self.getAdjacentPositions(node.getX(), node.getY())

        for adjNode in listAdjNodes:
            cost = self.calculateCostToObjective(node,adjNode,objective)
            self.currentMap[adjNode.getX()][adjNode.getY()].setCostToGoal(cost)
            ListOfNodes.append(self.currentMap[adjNode.getX()][adjNode.getY()])

        return ListOfNodes

    """calcula el costo de un nodo al objetivo utilizando la formula f(n) = g(n) + h(n)"""
    def calculateCostToObjective(self,currentNode,pos1, pos2):
        #el costo del origen al nodo actual
        costOriginToNode= self.calculateDistanceToAnObjective(self.origin,currentNode)
        #costo de nodo actual al objetivo
        costNodeToNode= self.calculateDistanceToAnObjective(pos1,pos2)
        #se suma 1 porque se dio un paso del nodo al adyacente.
        gn=costOriginToNode+1
        hn=costNodeToNode
        return gn + hn
    """Usamos manhattan para calcular la distancia entre dos nodos"""
    def calculateDistanceToAnObjective(self, Position1, Position2):
        return manhattan(Position1, Position2)




    """aqui usamos una inspiracion del algoritmo de costo pero para calcular a cual almacen nos conviene ir primero, este sera el objetivo
    obtendra los nodos adyecentes del nodo y sacara el calculo del costo a cada almacen por cada nodo adyacente"""
    def SearchShelfNodeWithLessCostAllAdjacent(self,node):
        listAdjNodes= self.getAdjacentPositions(node.getX(),node.getY())
        minorCost= 10000000
        nodeToMove = ""
        for adjNode in listAdjNodes:
            nodeToMoveCal,minorCostAdjNode = self.SearchShelfNodeWithLessCost(node,adjNode,minorCost)
            if minorCostAdjNode < minorCost:
                minorCost = minorCostAdjNode
                nodeToMove = nodeToMoveCal
        return self.mapObjetives[nodeToMove]

    """ aqui ya tenemos el nodo adyacente, vamos a calcular el costo de este nodo a cada uno de los almacenes y regresaremos el almacen con menor cotos"""
    def SearchShelfNodeWithLessCost(self,current,position,minorCost):
        mapCosts= self.calculateCostToShelfs(current,position)
        nodeToMove = ""
        for key, value in mapCosts.items():
            if value < minorCost:
                nodeToMove=key
                minorCost = value
        return (nodeToMove,minorCost)

    """Aqui calculamos el costo con una adaptacion a la formula donde incluimos el costo al objetivo del almacen f(n) = g(n) + h(n) + j(n)"""
    def calculateCostToShelfs(self,current,pos):
        mapCosts = {}
        mapDistancesNodeToShelfs = self.calculateDistanceToAllShelfs(pos)

        #le añadimos 1 por el costo del nodo actual al adyacente
        costOriginToNode= self.calculateDistanceToAnObjective(self.origin,current)
        costCurrentToNode= self.calculateDistanceToAnObjective(current,pos)
        for value in self.mapObjetives.values():
            mapCosts[value.getName()] = costOriginToNode + costCurrentToNode  + mapDistancesNodeToShelfs[value.getName()] +  self.mapDistancesShelfsToTheirObjective[value.getName()]
        return mapCosts

    """obtenemos un mapa de distancia del nodo a cada almacen"""
    def calculateDistanceToAllShelfs(self, pos):
        mapDistance = {}
        for value in self.mapObjetives.values():
            mapDistance[value.getName()] = manhattan(pos, value.currentPosition)
        return mapDistance


    """funcion para imprimis las listas cerradas y abiertas"""
    def printLists(self):
        print(f"                 \tx, y                              ")
        for node in self.openList:
            print(
                f"OPEN LIST Node : \t{node.getPosition().getX()}, {node.getPosition().getY()} cost: {node.costToGoal}")
        for node in self.closedList:
            print(
                f"CLOSED LIST Node : \t{node.getPosition().getX()}, {node.getPosition().getY()} cost: {node.costToGoal}")

    #actualizar el mapa para el recorrido
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

    #actualizar el mapa si encontro un almacen
    def updateMapRobotShelf(self,nodeToGo,shelf):
        for i in range(self.X):
            for j in range(self.Y):
                if self.currentMap[i][j].name == shelf.name:
                    self.currentMap[i][j].name = "*"
                    self.currentMap[i][j].type = ""
                if self.currentMap[i][j].getPosition().getX() == nodeToGo.getPosition().getX() and self.currentMap[i][j].getPosition().getY() == nodeToGo.getPosition().getY():
                    self.currentMap[i][j].name = shelf.name
                    self.currentMap[i][j].type = shelf.type

    """imprime los mapas dependiendo del caso"""

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


    """empezaremos el algoritmo de A star, hacemos un precalculo donde calcularemos el nodo actual, el almacen al cual ir
    añadimos a la lista abierta nuestro primer nodo, al track y al origen y empezamos con el algoritmo para buscar un almacen
    """
    def startAlgorithm(self):
        self.set_state("SearchShelf")
        currentNode= self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()]
        pos = currentNode.getPosition()
        ShelfToTake = self.SearchShelfNodeWithLessCost(pos,pos,10000)
        shelfNode= self.mapObjetives[ShelfToTake[0]]
        currentNode.setCostToGoal(self.calculateDistanceToAnObjective(currentNode.getPosition(), shelfNode.getPosition()))
        self.openList.append(self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()])
        self.track.append(currentNode)
        self.origin=currentNode.getPosition()
        self.A_star_algorithm_SearchShelf(currentNode)

        """algorimto para buscar un almacen """
    def A_star_algorithm_SearchShelf(self, currentNode):
        self.currentPosition = currentNode.getPosition()

        """si no hay almacenes que buscar se dara por terminado"""
        if  not self.mapObjetives:
            print("End of algorithm!!!!")
        else:
            """si encontro un almacen y este aun no esta en su objetivo se pasara al algoritmo de A start, mover almacen, 
            se limpiaran las listas y se hara un calculo para obtener los valores al nuevo objetivo"""
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
                self.origin = currentNode.getPosition()
                self.A_star_algorithm_MoveShelf(currentNode,self.mapObjetives[currentNode.name])

                """si es un obstaculo, se regresara al nodo anterior, se agregara a la lista cerrada, se sacara de la lista abierta"""
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
                """se asume que el robot puede pasar debajo de los almacenes que ya estan en su posicion
                se calculara el almcane con menor costo al cual ir, ese sera el objetivo
                se calculara el costo de los nodos adyacentes
                se sacaran los nodos que esten en la lista cerrada 
                se añadira el nodo actual a la lista cerrada
                se añadira los nodos adyacentes a la lista abierta por orden de menor a mayor costo
                se eligira el primer nodo de la lista abierta
                si la lista abierta esta vacia, se asume que ya no hay camino
                si el nodo al cual ir no es parte de los nodos adyacente, se asume que esta en un bloqueo y debera
                regresar por su camino recorrido hasta encontrar a ese nodo 
                si no, se tomara el nodo adyacente como el siguiente
                
                """
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


    """el algoritmo es escencialmente el mismo, con la diferencia que no calculara el almacen al cual conviene ir, sino al objetivo del
    almacen ya seleccionado, como estara cargando un almacen, aqui ya se tomaran como obstaculos,y al terminar de llevar el almacen al objetivo
    volvera a activar el algoritmo para encontrar el siguiente almacen"""
    def A_star_algorithm_MoveShelf(self, currentNode,Shelf):
            self.currentPosition = currentNode.getPosition()
            #self.track.append(currentNode)


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
                    ShelfToTake = self.SearchShelfNodeWithLessCost(pos,pos, 10000)
                    shelfNode = self.mapObjetives[ShelfToTake[0]]
                    currentNode.setCostToGoal(
                        self.calculateDistanceToAnObjective(currentNode.getPosition(), shelfNode.getPosition()))
                    self.openList.append(self.currentMap[self.currentPosition.getX()][self.currentPosition.getY()])
                    self.track.append(currentNode)
                    self.updateMapRobot(currentNode)
                    self.printCurrentMapObjective(currentNode)
                    print("\t\t start A* to search a Shelf\t\t")
                    self.origin=currentNode.getPosition()
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
