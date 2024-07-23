import math
from Position import Position


def manhattan_distance(pos1,pos2):
    return abs(pos2.getX() - pos1.getX()) + abs(pos2.getY() - pos1.getY())