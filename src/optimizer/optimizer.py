import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
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


def hinge(likelihood): # Doesn't need to be differentiable because it's not used in gradient descent, only in the final loss calculation
    return max(likelihood, -OUTLIER_MAX)

# -------- objective function --------------------
def log_loss(x, E_k, y_j, clauses, print_res=False, bounded=BOUND_LIKELIHOOD, use_prior=True):
    alpha_inv = x[: len(x)//2]      # values of alpha (slope) for each conditional structure
    x_0 = x[len(x)//2 :]            # center of logistic function for each conditional structure

    # Calculate log loss: based on data
    log_loss = 0
    for i in range(len(y_j)):
        # Calculate likelihood for base clause
        log_likelihood = - sp.logsumexp([0, - 1.0/alpha_inv[0] * (E_k[0][i] - x_0[0])])

        for j in range(len(clauses)):
            # Calculate likelihood for other clause
            log_likelihood_j = - sp.logsumexp([0, - 1.0/alpha_inv[j+1] * (E_k[j+1][i] - x_0[j+1])])

            if clauses[j] == 0: # AND: multiply likelihoods
                log_likelihood += log_likelihood_j 
            if clauses[j] == 1: # OR: add likelihoods
                log_likelihood = sp.logsumexp([log_likelihood, log_likelihood_j, log_likelihood+log_likelihood_j], b=[1, 1, -1])

            
        # Compute total log loss
        if y_j[i]:  # Satisfied transition
            if bounded:
                log_likelihood=hinge(log_likelihood)
            log_loss -= log_likelihood
        else:       # Unsatisfied transition
            log_not = sp.logsumexp([0, log_likelihood], b=[1, -1])
            if bounded:
                log_not=hinge(log_not)
            log_loss -= log_not
    
    if print_res:
        print("overall LL "+str(log_loss))

    # Calculate parameter loss: based on prior
    param_loss = 0
    if use_prior:
        for inv in alpha_inv:
            param_loss += inv * inv * ALPHA_LOSS_LOWER
            param_loss += 1.0/inv * 1.0/inv * ALPHA_LOSS_UPPER
        for val in x_0:
            param_loss += abs(val) * X_0_LOSS

    return log_loss / len(y_j) + param_loss



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

# ---------- Bounds ------------------------
class Bounds:
    def __init__(self, xmin, xmax):
        self.xmax = np.array(xmax)
        self.xmin = np.array(xmin)
    def __call__(self, **kwargs):
        x = kwargs["x_new"]
        tmax = bool(np.all(x <= self.xmax))
        tmin = bool(np.all(x >= self.xmin))
        return tmax and tmin


# Finds the minimum and maximum values where a change in state occurs, plus some extension
def find_min_max(expression, y_j):
    
    lo = min(expression)
    lo_ind = np.argmin(expression)
    hi = max(expression)
    hi_ind = np.argmax(expression)

    lo_diff = hi
    hi_diff = lo

    for i in range(len(y_j)):
        if not (y_j[i] == y_j[lo_ind]):
            lo_diff = min(lo_diff, expression[i])
        if not (y_j[i] == y_j[hi_ind]):
            hi_diff = max(hi_diff, expression[i])
    
    if lo_diff > hi_diff:
        # Swap
        temp_diff = lo_diff
        lo_diff = hi_diff
        hi_diff = temp_diff

    return (lo_diff - extension(lo_diff, hi_diff), hi_diff + extension(lo_diff, hi_diff))

# --------- Stepping (basin hopping) -------------------------
class TakeStep:
    def __init__(self, stepsize=0.5):
        self.stepsize = stepsize
        self.rng = np.random.default_rng()
    def __call__(self, x):
        mid = len(x) // 2
        s = self.stepsize
        x[:mid] += self.rng.uniform(-s, s, x[:mid].shape)
        x[mid:] += self.rng.uniform(-100*s, 100*s, x[mid:].shape)
        # x += (np.random.randint(2, size=x.shape)-0.5)*2 # Add some randomness (stochastic gradient descent)
        return x

# ---------- Optimizer ------------------------

# Runs the optimizer, given some initial parameters
def run_optimizer_from_initial(E_k, y_j, clauses, bounds, bounds_arr, bounds_obj, step_obj, init):
    extra_args = (E_k, y_j, clauses)
    minimizer_kwargs = {"method": "BFGS", "args" : extra_args}

    if(OPT_METHOD == 0):        # BFGS (Gradient descent) - local optimization
        res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='BFGS', options={'maxiter': MAX_ITER, 'disp': False})
    elif(OPT_METHOD == 1):      # L-BFGS-B (Gradient descent) - local optimization
        if BOUND_ALPHA:
            res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='L-BFGS-B', options={'maxiter': MAX_ITER, 'disp': False, 'eps': 1e-2}, bounds=bounds, tol=1e-16)
        else:
            res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='L-BFGS-B', options={'maxiter': MAX_ITER, 'disp': False})
    elif(OPT_METHOD == 2):      # Basin hopping - global optimization
        res = optimize.basinhopping(log_loss, init,
                                niter=MAX_ITER, T=100.0,
                                minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, 
                                take_step=step_obj, callback=print_fun)
    elif(OPT_METHOD == 3):      # Dual annealing - global optimization
        res = optimize.dual_annealing(log_loss, bounds, x0=init, 
                                        maxiter=MAX_ITER, initial_temp=50000, 
                                        visit=3.0, accept=-5, 
                                        minimizer_kwargs=minimizer_kwargs)
    elif(OPT_METHOD == 4):      # DIRECT - global optimization
        res = optimize.direct(log_loss, bounds_arr, args=extra_args, maxiter=MAX_ITER)
    else:
        sys.exit("Please use a valid optimization method")

    print_with_padding("Optimal parameters", "|")
    debug(res.x)
    print_with_padding("Num iterations", res.nit)
    print_with_padding("Minimum value", res.fun)
    debug("")

    # Remove NaNs
    res.fun = np.nan_to_num(res.fun, nan=float("inf"))

    return res

# Handles initialization and enumeration, then calls the optimizer
def run_optimizer(queue, index, E_k, y_j, clauses):

    if(not PRINT_WARNINGS):
        warnings.filterwarnings('ignore')

    # Sampling: take a subset of the given expressions to optimize over
    count_yes = 0
    count_no = 0
    E_k_subset = []
    y_j_subset = []
    for _ in range(0, len(E_k)):
        E_k_subset.append([])
    
    for i in range(0, len(y_j)):
        if y_j[i] and count_yes < MAX_EX_YES:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            count_yes += 1
        if not y_j[i] and count_no < MAX_EX_NO:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            count_no += 1

    # ---------- Bounds --------------------
    # Custom step function
    step_obj = TakeStep()

    # Bounds on 1/alpha : defined by MIN_ALPHA
    alpha_bounds = [(-1.0/MIN_ALPHA, 1.0/MIN_ALPHA) for expression in E_k_subset]

    # Bounds on randomly initialized value for x_0: calculated from minimum/maximum and provided bounds extension
    x_0_init_bounds = [find_min_max(expression, y_j_subset) for expression in E_k_subset]
    # Bounds on x_0 itself during optimization: is infinity because we need some variables to be unbounded for LBFGS-B to work
    x_0_bounds = [(-np.inf, np.inf) for expression in E_k_subset]

    # Setup bounds
    bounds = np.concatenate((alpha_bounds, x_0_bounds))
    bounds_lower = [b[0] for b in bounds]
    bounds_upper = [b[1] for b in bounds]
    bounds_obj = Bounds(bounds_lower, bounds_upper)
    bounds_arr = optimize.Bounds(bounds_lower, bounds_upper)

    print_with_padding("Bounds", "|")
    debug(bounds)

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
            alpha_init = []
            for i in range(len(E_k_subset)):
                alpha_init.append(1.0/INIT_ALPHA if (signs & (1 << i)) else -1.0/INIT_ALPHA)

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
    partial_optimizer = functools.partial(run_optimizer_from_initial, E_k_subset, y_j_subset, clauses, bounds, bounds_arr, bounds_obj, step_obj)
    with multiprocessing.Pool(NUM_CORES) as p:
        output = p.map(partial_optimizer, input)
    
    bestRes = min(output, key=lambda i: i.fun)
    

    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value - training set", bestRes.fun)
    # Run again with the validation set and no prior
    bestRes.fun = log_loss(bestRes.x, E_k, y_j, clauses, False, True, False)
    print_with_padding("Minimum value - validation set", bestRes.fun)

    # takes the inverse of inv_alpha to get alpha to pass into EMDIPS
    for i in range(len(bestRes.x[: len(bestRes.x)//2])):
        bestRes.x[i]=1.0/bestRes.x[i]

    if (not queue is None):
        queue.put( (index, (bestRes.fun, list(bestRes.x)) ) )

    return (bestRes.fun, list(bestRes.x))


# Run the optimizer on multiple expression examples in parallel
def run_optimizer_threads(E_k_arr, y_j, clauses_arr):
    q = multiprocessing.Queue()
    processes = []
    for i in range(len(E_k_arr)):
        p = multiprocessing.Process(target=run_optimizer, args=(q, i, E_k_arr[i], y_j, clauses_arr[i]))
        processes.append(p)
        p.start()
    
    results = [None] * len(processes)
    for p in processes:
        tup = q.get()
        results[tup[0]] = tup[1]
    for p in processes:
        p.join()

    return results