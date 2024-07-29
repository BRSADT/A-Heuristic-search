import math
from Position import Position


def manhattan_distance(pos1,pos2):
    return abs(pos2.getX() - pos1.getX()) + abs(pos2.getY() - pos1.getY())

def addNodeIfNotInList(nodesToAdd, nodeList):
    for nodeToAdd in nodesToAdd:
        found=False
        for node in nodeList:
            if node.getPosition().getX()== nodeToAdd.getPosition().getX() and node.getPosition().getY()== nodeToAdd.getPosition().getY():
                found=True
        if found==False:
            nodeList.append(nodeToAdd)
    return nodeList


def removeNodeIfInList(nodes, nodeList):
    list=[]
    for nodeToAdd in nodes:
        found=False
        for node in nodeList:
            if node.getPosition().getX()== nodeToAdd.getPosition().getX() and node.getPosition().getY()== nodeToAdd.getPosition().getY():
                found=True
        if found==False:
            list.append(nodeToAdd)
    return list

def removeElementInList(nodeToRemove, nodeList):
    list=[]

    found=False
    for node in nodeList:
        if node.getPosition().getX()== nodeToRemove.getPosition().getX() and node.getPosition().getY()== nodeToRemove.getPosition().getY():
            found=True
        else:
            list.append(node)
    return list


def isNodeInList(nodeToCheck, nodeList):
    for node in nodeList:
        if node.getPosition().getX() == nodeToCheck.getPosition().getX() and node.getPosition().getY()== nodeToCheck.getPosition().getY():
            return True
    return False

def updateNodeIfInList(nodesToAdd, nodeList):
    reversedList=list(reversed(nodesToAdd))
    for nodeToAdd in reversedList:
        found=False
        for node in nodeList:
            if node.getPosition().getX()== nodeToAdd.getPosition().getX() and node.getPosition().getY()== nodeToAdd.getPosition().getY():
               node.setCostToGoal(nodeToAdd.getCostToGoal())
               found=True
        if found==False:
            nodeList.insert(0,nodeToAdd)
    return nodeList
def convertMapTostring(currentMap,x,y):
    list = [[0 for j in range(y)] for i in range(x)]
    for i in range(x):
        for j in range(y):
            list[i][j]=currentMap[i][j].getName()
    return list