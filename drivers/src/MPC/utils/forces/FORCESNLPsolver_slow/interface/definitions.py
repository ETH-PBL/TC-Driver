import numpy
import ctypes

name = "FORCESNLPsolver_slow"
requires_callback = True
lib = "lib/libFORCESNLPsolver_slow.so"
lib_static = "lib/libFORCESNLPsolver_slow.a"
c_header = "include/FORCESNLPsolver_slow.h"
nstages = 21

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("xinit"               , "dense" , ""               , ctypes.c_double, numpy.float64, (  8,   1),    8),
 ("x0"                  , "dense" , ""               , ctypes.c_double, numpy.float64, (252,   1),  252),
 ("all_parameters"      , "dense" , ""               , ctypes.c_double, numpy.float64, (315,   1),  315)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("x01"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x02"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x03"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x04"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x05"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x06"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x07"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x08"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x09"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x10"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x11"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x12"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x13"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x14"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x15"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x16"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x17"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x18"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x19"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x20"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12),
 ("x21"                 , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),   12)]

# Info Struct Fields
info = \
[('it', ctypes.c_int),
('it2opt', ctypes.c_int),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('rsnorm', ctypes.c_double),
('rcompnorm', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int),
('lsit_cc', ctypes.c_int),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double),
('fevalstime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4), 
	(12, 8, 4, 15, 12, 12, 0, 4)
]