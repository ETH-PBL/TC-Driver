import numpy as np

class Constraint():

    def __init__(self) -> None:
        pass

class BoundaryConstraint(Constraint):

    def __init__(self, dim:int, stage: int, lower: np.float32=-np.inf, upper: np.float32=np.inf) -> None:
        super().__init__()
        self.lower = lower
        self.upper = upper
        self.dim = dim 
        self.stage = stage 

    def __repr__(self) -> str:
        lower_str = ""
        upper_str = ""
        if self.lower != -np.inf:
            lower_str = str(self.lower) + " <= "
        if self.upper != np.inf:
            upper_str = " <= " + str(self.upper)
        
        return lower_str + "x_{}({})".format(self.dim, self.stage) + upper_str

class LinearConstraint(Constraint):

    def __init__(self) -> None:
        super().__init__()