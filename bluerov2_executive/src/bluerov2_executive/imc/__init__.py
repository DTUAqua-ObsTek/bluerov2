from enum import IntEnum, auto, Enum
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
# Ozer Ozkahraman (ozero@kth.se)

########################
# IMC Enums
# See https://github.com/LSTS/imc/blob/master/IMC.xml
########################
class State(IntEnum):
    STATE_BLOCKED = 0
    STATE_READY = 1
    STATE_INITIALIZING = 2
    STATE_EXECUTING = 3

class OpMode(IntEnum):
    OP_MODE_SERVICE = 0
    OP_MODE_CALIBRATION = 1
    OP_MODE_ERROR = 2
    OP_MODE_MANEUVER = 3
    OP_MODE_EXTERNAL = 4
    OP_MODE_BOOT = 5

class PlanDBType(IntEnum):
    PLANDB_TYPE_REQUEST = 0
    PLANDB_TYPE_SUCCESS = 1
    PLANDB_TYPE_FAILURE = 2
    PLANDB_TYPE_IN_PROGRESS = 3

class PlanDBOp(IntEnum):
    PLANDB_OP_SET = 0
    PLANDB_OP_DEL = 1
    PLANDB_OP_GET = 2
    PLANDB_OP_GET_INFO = 3
    PLANDB_OP_CLEAR = 4
    PLANDB_OP_GET_STATE = 5
    PLANDB_OP_GET_DSTATE = 6
    PLANDB_OP_BOOT = 7

# TODO eventually implement other types of maneuvers
# see mission_plan -> read_plandb
class Maneuver(IntEnum):
    MANEUVER_GOTO = 450
    MANEUVER_SAMPLE = 489


# Speed units
class SpeedUnits(IntEnum):
    SPEED_UNIT_MPS = 0
    SPEED_UNIT_RPM = 1
    SPEED_UNIT_PERCENTAGE = 2

# These are the same in smarc_msgs/GotoWaypoint.action
# Z units
class ZUnits(IntEnum):
    Z_NONE = 0
    Z_DEPTH = 1
    Z_ALTITUDE = 2
    Z_HEIGHT = 3

# a list of actions that we will consider
# as the imc 'executing' state
class ActionNames(Enum):
    GOTOWAYPOINT = "A_GotoWaypoint"
    FOLLOWLEADER = "A_FollowLeader"
    SETNEXTPLANACTION = "A_SetNextPlanAction" #so we dont spam service/maneuver when going tru a lot of waypoints quickly

# same thing for the 'blocked' state
class BlockedActionNames(Enum):
    EMERGENCYSURFACE = "A_EmergencySurface"
