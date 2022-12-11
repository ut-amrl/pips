import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import math

from scipy import optimize
import warnings
import time

import optimizer

# -------- Tests, Problems, and Data -----------------------------

### Goal: synthesize x < 30 ----------> SUCCESS
# https://www.desmos.com/calculator/rymyh1m1gv

clauses = []
y_j = [True, False]
E_k = [
    [29, 31]
]

### Tests
# assert abs(optimizer.log_loss([2, 25]) - 12) < 0.001
# assert abs(optimizer.log_loss([4, 35]) - 24) < 0.001

# res = optimizer.run_optimizer(None, E_k, y_j, clauses)
# assert abs(res.x[1] - 30) < 0.05



### Goal: synthesize (x > 10 && x < 20) ----------> SUCCESS

clauses = [ 0 ] # (p_1 & p_2)
y_j = [False, False, False, True, True, True, True, False, False, False, False] # whether or not each example satisfied the transition
E_k = [
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49],
] # value of E_k(s) for each example, for each predicate

### Tests
# res = optimizer.run_optimizer(None, E_k, y_j, clauses)








### Goal: synthesize (x > 10 && x < 20) || x > 50 ----------> SUCCESS (after tweaking)
# https://www.desmos.com/calculator/1m79wd5h0e
# https://www.desmos.com/calculator/ttucfvkvlp


# ---------- Define examples -------------------------------------
y_j = [False, False, False, True, True, True, True, False, False, False, False, True, True, True] # whether or not each example satisfied the transition
E_k = [ 
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
] # value of E_k(s) for each example, for each predicate

# now introduce some error
# E_k = [
#     [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
#     [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
#     [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
# ] # value of E_k(s) for each example, for each predicate

# ---------- Define equation ------------------------------------
clauses = [ 0 , 1 ]     # (p_1 & p_2) | p_3

### ----------------- Tests ---------------------------------------
# assert abs(optimizer.log_loss([2, -2, 1, 12, 18, 50]) - 4.9155) < 0.001
# assert abs(optimizer.log_loss([-1, -5, -1, 3, 18, 4]) - 176.66) < 0.001
# assert abs(optimizer.log_loss([-0.09, 0.3, 0.3, 30, 3, 53]) - 6.7191) < 0.001
# assert abs(optimizer.log_loss([3, -3, 8, 10, 20, 50]) - .195) < 0.001
# assert abs(optimizer.log_loss([ 9.17704836, -9.96464641,  9.30102843, 10.07471293, 20.03487693, 49.95211582]) - 0) < 0.001

# start = time.perf_counter()
# res = optimizer.run_optimizer(None, E_k, y_j, clauses)
# end = time.perf_counter()

# optimizer.print_with_padding("Time Elapsed", end-start)





### Goal: synthesize (x > 10 && x < 20) || x > 50 IN PARALLEL, WITH DIFFERENT DATA
# ---------- Define examples -------------------------------------
y_j = [False, False, False, True, True, True, True, False, False, False, False, True, True, True] # whether or not each example satisfied the transition
E_k = [
    [ 
        [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
        [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
        [-5, 0, 9, 11, 14, 15, 19, 21, 30, 40, 49, 51, 55, 70],
    ], # without error
    [
        [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
        [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
        [-5, 0, 9, 11, 14, 35, 19, 21, 30, 40, 49, 51, 55, 70],
    ] # with error
]

# ---------- Define equation ------------------------------------
# clauses = [     #  (p_1 & p_2) | p_3
#     [0, 1],
#     [0, 1]
# ]

# # start = time.perf_counter()
# # res = optimizer.run_optimizer_threads(E_k, y_j, clauses)
# # end = time.perf_counter()

# # optimizer.print_with_padding("Time Elapsed", end-start)


# E_k = [[14.0625, 16.0, 10.0, 66.66666412353516, 16.0, 10.0, 66.66666412353516, 14.0625]]
# y_j = [1, 1, 1, 0, 0, 0, 0, 0]
# clauses = []


# start = time.perf_counter()
# res = optimizer.run_optimizer(None, E_k, y_j, clauses)
# end = time.perf_counter()

# optimizer.print_with_padding("Time Elapsed", end-start)



# ---------- Define equation ------------------------------------


E_k = [[
    [-0.93,
    1.7,
    0.1,
    -0.1,
    1.7,
    -3.7,
    -55.3,
    0,
    -55.3,
    -2.4,
    0.1,
    0,
    -2.39,
    -3.72,
    -0.93,
    -0.1,
    -0.94,
    0.1,
    1.7,
    -55.3,
    -9,
    -14.34,
    -15.3,
    -26.36,
    -79.2,
    -14.1,
    -0.2,
    1.2,
    -18.55,
    -14.34],
    [29.34,
    10.55,
    15.815,
    178.1,
    10.54,
    -11.8,
    -3.99,
    131.35,
    -4,
    -3.7,
    15.815,
    131.35,
    -3.735,
    -11.8,
    29.34,
    178.12,
    29.34,
    15.815,
    10.54,
    -4,
    -3.2,
    89.3,
    174.2,
    292.2,
    299.68,
    153,
    14,
    11.78,
    224.25,
    89.3]
]]
y_j = [1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
clauses = [[1]]

start = time.perf_counter()
res = optimizer.run_optimizer_threads(E_k, y_j, clauses)
print(res)
end = time.perf_counter()

optimizer.print_with_padding("Time Elapsed", end-start)