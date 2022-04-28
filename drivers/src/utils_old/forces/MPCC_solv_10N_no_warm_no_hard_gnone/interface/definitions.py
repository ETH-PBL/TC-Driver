import numpy
import ctypes

name = "MPCC_solv_10N_no_warm_no_hard_gnone"
requires_callback = False
lib = "lib/libMPCC_solv_10N_no_warm_no_hard_gnone.so"
lib_static = "lib/libMPCC_solv_10N_no_warm_no_hard_gnone.a"
c_header = "include/MPCC_solv_10N_no_warm_no_hard_gnone.h"
nstages = 11

# Parameter             | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
params = \
[("c_1"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  8,   1),    8),
 ("c_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("c_11"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,   1),   10),
 ("H_1"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("H_11"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,  14),  196),
 ("f_1"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("f_11"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 14,   1),   14),
 ("C_1"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("C_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, ( 10,  14),  140),
 ("A_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("A_11"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,  14),   28),
 ("b_2"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_3"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_4"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_5"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_6"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_7"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_8"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_9"                 , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_10"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2),
 ("b_11"                , "dense" , ""               , ctypes.c_double, numpy.float64, (  2,   1),    2)]

# Output                | Type    | Scalar type      | Ctypes type    | Numpy type   | Shape     | Len
outputs = \
[("X"                   , ""      , ""               , ctypes.c_double, numpy.float64,     ( 12,),  132)]

# Info Struct Fields
info = \
[('it', ctypes.c_int32),
('it2opt', ctypes.c_int32),
('res_eq', ctypes.c_double),
('res_ineq', ctypes.c_double),
('pobj', ctypes.c_double),
('dobj', ctypes.c_double),
('dgap', ctypes.c_double),
('rdgap', ctypes.c_double),
('gradient_lag_norm', ctypes.c_double),
('mu', ctypes.c_double),
('mu_aff', ctypes.c_double),
('sigma', ctypes.c_double),
('lsit_aff', ctypes.c_int32),
('lsit_cc', ctypes.c_int32),
('step_aff', ctypes.c_double),
('step_cc', ctypes.c_double),
('solvetime', ctypes.c_double)
]

# Dynamics dimensions
#   nvar    |   neq   |   dimh    |   dimp    |   diml    |   dimu    |   dimhl   |   dimhu    
dynamics_dims = [
	(14, 8, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0), 
	(14, 10, 0, 0, 12, 12, 0, 0)
]