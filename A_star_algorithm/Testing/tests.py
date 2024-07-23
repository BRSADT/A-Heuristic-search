import pytest

from A_Star_Algorithm import A_Star_Algorithm
from Position import Position


def test_adj():
    a = A_Star_Algorithm()
    a.setScenario()
    list = a.getAdjacentPositions(0, 3)
    assert len(list)==2

    assert list[0].getX() == 1
    assert list[0].getY() == 3

    assert list[1].getX() == 0
    assert list[1].getY() == 2

    list = a.getAdjacentPositions(2, 2)
    assert len(list) == 4

    assert list[0].getX() == 1
    assert list[0].getY() == 2

    assert list[1].getX() == 2
    assert list[1].getY() == 3

    assert list[2].getX() == 3
    assert list[2].getY() == 2

    assert list[3].getX() == 2
    assert list[3].getY() == 1

def test_Distances():
    a = A_Star_Algorithm()
    a.setScenario()
    assert a.mapDistancesShelfsToTheirObjective["M1"]==6
    assert a.mapDistancesShelfsToTheirObjective["M2"]==3
    assert a.mapDistancesShelfsToTheirObjective["M3"]==5

    mapDistances= a.calculateDistanceToAllShelfs(Position(2,1))

    assert mapDistances["M1"] ==3
    assert mapDistances["M2"] == 1
    assert mapDistances["M3"] == 4



    assert a.calculateDistanceToAnObjective(Position(2,0), Position(1,0)) == 1

def test_Costs():
    a = A_Star_Algorithm()
    a.setScenario()



    mapCosts= a.calculateCostToShelfs(Position(2,1))
    assert mapCosts["M1"] ==10
    assert mapCosts["M2"] == 5
    assert mapCosts["M3"] == 10

    mapCosts= a.calculateCostToShelfs(Position(1,2))
    assert mapCosts["M1"] ==10
    assert mapCosts["M2"] == 7
    assert mapCosts["M3"] == 8

    mapCosts= a.calculateCostToShelfs(Position(2,3))
    assert mapCosts["M1"] ==12
    assert mapCosts["M2"] == 7
    assert mapCosts["M3"] == 8

    mapCosts = a.calculateCostToShelfs(Position(3, 2))
    assert mapCosts["M1"] == 12
    assert mapCosts["M2"] == 7
    assert mapCosts["M3"] == 10

    assert a.calculateCostToObjective(Position(1,0), Position(3,2)) == 7

def test_searchNode():
    a = A_Star_Algorithm()
    a.setScenario()
    Node = a.SearchShelfStateGetNextNode(Position(2,2))
    assert Node == "M2"
def test_Scenario():
    a = A_Star_Algorithm()
    a.setScenario()
    scenario=a.currentMap
    assert scenario[0][0].name == "M1"
    assert scenario[0][0].type == "Shelf"
    assert scenario[0][1].name == "#"
    assert scenario[0][1].type == "Obstacle"
    assert scenario[0][2].name == ""
    assert scenario[0][2].type == ""
    assert scenario[0][3].name == "M3"
    assert scenario[0][3].type == "Shelf"

    assert scenario[1][0].name == ""
    assert scenario[1][0].type == ""
    assert scenario[1][1].name == "#"
    assert scenario[1][1].type == "Obstacle"
    assert scenario[1][2].name == ""
    assert scenario[1][2].type == ""
    assert scenario[1][3].name == ""
    assert scenario[1][3].type == ""

    assert scenario[2][0].name == "M2"
    assert scenario[2][0].type == "Shelf"
    assert scenario[2][1].name == ""
    assert scenario[2][1].type == ""
    assert scenario[2][2].name == "R"
    assert scenario[2][2].type == "Robot"
    assert scenario[2][3].name == ""
    assert scenario[2][3].type == ""

    assert scenario[3][0].name == ""
    assert scenario[3][0].type == ""
    assert scenario[3][1].name == ""
    assert scenario[3][1].type == ""
    assert scenario[3][2].name == ""
    assert scenario[3][2].type == ""
    assert scenario[3][3].name == ""
    assert scenario[3][3].type == ""

    assert len(a.mapObjetives) == 3

    assert  a.mapObjetives["M1"].getGoalPosition().getX() == 3
    assert  a.mapObjetives["M1"].getGoalPosition().getY() == 3

    assert  a.mapObjetives["M2"].getGoalPosition().getX() == 3
    assert  a.mapObjetives["M2"].getGoalPosition().getY() == 2

    assert  a.mapObjetives["M3"].getGoalPosition().getX() == 3
    assert  a.mapObjetives["M3"].getGoalPosition().getY() == 1

    assert a.currentPosition.getX() == 2
    assert a.currentPosition.getY() == 2