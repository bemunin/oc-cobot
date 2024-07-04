from enum import Enum


# Enums
class ObjectType(Enum):
    GEOMETRIC = 0
    REAL_LIFE = 1


class ObjectItem(Enum):
    ALL = "all"
    # Geometric Objects, prefix with "geo_<filename or asset path>"
    GEO_CUBE = "cube.usd"
    GEO_CYLINDER = "cylinder.usd"
    GEO_BAR = "bar.usd"
    # capsule shape has problem with contact, need to fix
    # GEO_CAPSULE = "capsule.usd"
    # Real Life Objects, prefix with "real_<filename or asset path>"
    REAL_BANANA = "banana.usd"
    REAL_MUSTARD_BOTTLE = "mustard_bottle.usd"
    REAL_CRACKER_BOX = "cracker_box.usd"
    REAL_TOMATO_SOUP_CAN = "tomato_soup_can.usd"
