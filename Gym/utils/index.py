from enum import IntEnum

class Index(IntEnum):
    S_X = 0
    S_Y = 1
    DELTA = 2
    V = 3
    PHI = 4
    PHI_DOT = 5
    BETA = 6
    THETA = 7
    SOFT = 8
    V_K = 9
    V_DELTA = 10
    ACC = 11
    PREV_V_DELTA = 12
    PREV_ACC = 13
    
class NlIndex(IntEnum):
    SOFT = 0
    V_K = 1
    V_DELTA = 2
    ACC = 3
    S_X = 4
    S_Y = 5
    DELTA = 6
    V = 7
    PHI = 8
    PHI_DOT = 9
    BETA = 10
    THETA = 11

class ParameterIndex(IntEnum):
    FXINT = 0
    FYINT = 1
    fINT = 2
    FXOUT = 3
    FYOUT = 4
    fOUT = 5
    XTH = 6
    YTH = 7
    PHITH = 8
    fINTHARD = 9
    fOUTHARD = 10
    dPHI = 11
    dX = 12
    dY = 13
    THETA = 14
