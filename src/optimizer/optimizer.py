import numpy as np
from scipy import optimize
from scipy import special as sp
import multiprocessing
from itertools import repeat
import functools

import math
import warnings
import time
import json

from optimizer_settings import *

PRINT_WARNINGS = True      # Debugging info
PRINT_PADDING = 30          # Output configuration
BOUNDS_EXTEND = 0.1

def hinge(likelihood): # Doesn't need to be differentiable because it's not used in gradient descent, only in the final loss calculation
    return max(likelihood, -OUTLIER_MAX)

# -------- objective function --------------------
def log_loss(x, E_k, y_j, clauses, use_prior=True):
    half = len(x) // 2

    log_loss = 0
    for i in range(len(y_j)):
        # Calculate likelihood for base clause
        log_likelihood = - np.logaddexp(0, - x[0] * (E_k[0][i] - x[half]))

        for j in range(len(clauses)):
            # Calculate likelihood for other clauses
            log_likelihood_j = - np.logaddexp(0, - x[j+1] * (E_k[j+1][i] - x[j+1+half]))
            if clauses[j] == 0:
                log_likelihood += log_likelihood_j
            else:
                a_max = np.logaddexp(log_likelihood, log_likelihood_j)
                if not np.isfinite(a_max):
                    a_max = 0
                log_likelihood = np.log(1 - np.exp(log_likelihood+log_likelihood_j - a_max)) + a_max
            
        # Compute total log loss
        log_loss -= (log_likelihood if y_j[i] else np.log(-np.expm1(log_likelihood-1E-10)))

    # Calculate parameter loss: based on prior
    return log_loss + (0 if not use_prior else np.sum([a * a * ALPHA_LOSS_UPPER for a in x[: half]]))

# ------- helper functions ----------------------
def extension(l, r): # list range
    return (max(l, r) - min(l, r)) * BOUNDS_EXTEND

def debug(str):
    if PRINT_DEBUG:
        print(str)

def print_with_padding(label, value):
    debug((label+" ").ljust(PRINT_PADDING, "-")+" "+str(value))

# ---------- Callback ------------------------
def print_fun(x, f, accepted):
    debug("at minimum %.4f accepted %d" % (f, int(accepted)))
    debug("with parameters: ")
    debug(x)
    return

# Finds the minimum and maximum values where a change in state occurs, plus some extension
def find_min_max(expression, y_j):
    
    lo_ind = np.argmin(expression)
    hi_ind = np.argmax(expression)

    lo_diff = max(expression)
    hi_diff = min(expression)

    for i in range(len(y_j)):
        if not (y_j[i] == y_j[lo_ind]):
            lo_diff = min(lo_diff, expression[i])
        if not (y_j[i] == y_j[hi_ind]):
            hi_diff = max(hi_diff, expression[i])
    
    if lo_diff > hi_diff:
        (lo_diff, hi_diff) = (hi_diff, lo_diff)

    return (lo_diff - extension(lo_diff, hi_diff), hi_diff + extension(lo_diff, hi_diff))

# ---------- Optimizer ------------------------

# Runs the optimizer, given some initial parameters
def run_optimizer_from_initial(E_k, y_j, clauses, init):
    extra_args = (E_k, y_j, clauses)
    minimizer_kwargs = {"method": "BFGS", "args" : extra_args}

    start = time.perf_counter()
    debug("start opt "+str(init))

    if(OPT_METHOD == 0):        # BFGS (Gradient descent) - local optimization
        res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='BFGS', options={'maxiter': MAX_ITER, 'disp': False})
    elif(OPT_METHOD == 1):      # L-BFGS-B (Gradient descent) - local optimization
        res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='L-BFGS-B', options={'maxiter': MAX_ITER, 'disp': False})
    elif(OPT_METHOD == 2):      # Basin hopping - global optimization
        res = optimize.basinhopping(log_loss, init, niter=MAX_ITER, T=100.0,
                                minimizer_kwargs=minimizer_kwargs, callback=print_fun)
    elif(OPT_METHOD == 3):      # Dual annealing - global optimization
        res = optimize.dual_annealing(log_loss, x0=init, 
                                        maxiter=MAX_ITER, initial_temp=50000, visit=3.0, accept=-5, minimizer_kwargs=minimizer_kwargs)
    elif(OPT_METHOD == 4):      # DIRECT - global optimization
        res = optimize.direct(log_loss, args=extra_args, maxiter=MAX_ITER)
    else:
        print("Please use a valid optimization method")

    print_with_padding("Optimal parameters", "|")
    debug(res.x)
    print_with_padding("Num iterations", res.nit)
    print_with_padding("Minimum value", res.fun)

    end = time.perf_counter()
    debug("time: "+str(end-start))

    debug("")

    # Remove NaNs
    res.fun = np.nan_to_num(res.fun, nan=float("inf"))

    return res

# Handles initialization and enumeration, then calls the optimizer
def run_optimizer(queue, index, E_k, y_j, clauses):
    E_k = np.array(E_k)
    y_j = np.array(y_j)
    clauses = np.array(clauses)

    if(not PRINT_WARNINGS):
        warnings.filterwarnings('ignore')

    # Sampling: take a subset of the given expressions to optimize over
    E_k_subset = []
    y_j_subset = []
    for _ in range(0, len(E_k)):
        E_k_subset.append([])
    
    yes_count = float(np.count_nonzero(y_j))
    no_count = float(len(y_j) - yes_count)

    if np.count_nonzero(y_j) == 0:
        # No examples -> just return false
        init = np.concatenate(([1 for _ in range(len(E_k_subset))], [1000000000 for _ in range(len(E_k_subset))]))
        
        print_with_padding("Final parameters", "|")
        debug("False")
        print_with_padding("Minimum value - training set", 0.0)
        print_with_padding("Minimum value - validation set", 0.0)

        if (not queue is None):
            queue.put( (index, (0, list(init)) ) )

        return (0, list(init))

    yes_count = np.rint(yes_count * EX_SAMPLED / len(y_j))
    no_count = np.rint(no_count * EX_SAMPLED / len(y_j))

    for i in range(0, len(y_j)):
        if y_j[i] and yes_count > 0:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            yes_count -= 1
        if not y_j[i] and no_count > 0:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            no_count -= 1
    
    # ---------- Bounds --------------------
    # Bounds on randomly initialized value for x_0: calculated from minimum/maximum and provided bounds extension
    x_0_init_bounds = [find_min_max(expression, y_j_subset) for expression in E_k_subset]

    # ---------- Initialization ------------------------
    input = []

    for iter in range(max(1, INITIAL_VALUES)):

        # Initialization of x_0

        if INITIAL_VALUES == 0 or iter == 0:
            x_0_init = np.zeros(len(E_k_subset))                                               # Initialize to 0s
        elif INITIAL_VALUES == 1 or iter == 1:
            x_0_init = [sum(expression)/len(expression) for expression in E_k_subset]          # Initialize to the average
        elif INITIAL_VALUES > 1:
            x_0_init = [np.random.uniform(bound[0], bound[1]) for bound in x_0_init_bounds]  # Initialize randomly 

        for signs in range(pow(2, len(E_k_subset))): # iterate over possible signs for alpha
            
            # Initialization of alpha
            alpha_init = [(INIT_ALPHA if (signs & (1 << i)) else -INIT_ALPHA) for i in range(len(E_k_subset))]
            if not ENUMERATE_SIGNS: # Initialize to 0 (don't iterate over signs)
                alpha_init = np.zeros(len(E_k_subset))
            init = np.concatenate((alpha_init, x_0_init))

            # Calling the optimizer
            input.append(init)
            
            if not ENUMERATE_SIGNS:
                break
        
        if INITIAL_VALUES <= 1:
            break
    
    print_with_padding("Initial values", "|")
    debug(input)
    
    # Run optimizer in parallel
    partial_optimizer = functools.partial(run_optimizer_from_initial, E_k_subset, y_j_subset, clauses)
    with multiprocessing.Pool(NUM_CORES) as p:
        output = p.map(partial_optimizer, input)
    bestRes = min(output, key=lambda i: i.fun)

    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value - training set", bestRes.fun)
    # Run again with the validation set and no prior
    bestRes.fun = log_loss(bestRes.x, E_k, y_j, clauses, False) / len(y_j)
    print_with_padding("Minimum value - validation set", bestRes.fun)

    if (not queue is None):
        queue.put( (index, (bestRes.fun, list(bestRes.x)) ) )

    return (bestRes.fun, list(bestRes.x))

def wrapper(target, args, sema):
    # Do work
    target(args[0], args[1], args[2], args[3], args[4])

    # Release semaphore
    sema.release()

# Run the optimizer on multiple expression examples in parallel
def run_optimizer_threads(E_k_arr, y_j, clauses_arr):
    if DEBUG:
        print("| Sampling " + str(EX_SAMPLED) + " examples")
    
    q = multiprocessing.Queue()
    processes = []
    sema = multiprocessing.Semaphore(BATCH_SIZE) # Limit the number of programs to BATCH_SIZE

    for i in range(len(E_k_arr)):
        # Acquire semaphore
        sema.acquire()

        # Start process
        p = multiprocessing.Process(target=wrapper, args=(run_optimizer, (q, i, E_k_arr[i], y_j, clauses_arr[i]), sema))
        processes.append(p)
        p.start()
    
    results = [None] * len(processes)
    for p in processes:
        tup = q.get()
        results[tup[0]] = tup[1]
    for p in processes:
        p.join()

    return results