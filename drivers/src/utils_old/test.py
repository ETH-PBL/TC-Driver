import numpy as np
from numba import jit

@jit(nopython=True)
def foo():
    A = np.array([[0.0, 0], [1, 5*np.pi]], dtype = np.float64)
    

if __name__ == "__main__":
    
    foo()
