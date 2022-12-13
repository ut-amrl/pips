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


# ------- Parameters -----------------------------
opt_method = 1                  # optimization method, See below
enumerateSigns = True           # Equivalent to enumerating over > and <
print_debug = False             # Extra debugging info
initial_values = 8              # Initial values for x_0: 0 = all zeros, 1 = average, >1 = do all of the above, then enumerate over random initial guesses (use this to specify how many)
num_cores = 4                   # Number of processes to run in parallel
min_alpha = 1.0                 # lowest slope allowed
initial_alpha = 1.0             # starting slope
bound_alpha = True              # whether to bound alpha (to ensure slope is not too low)
bounds_extension = 0.1          # Amount to search above and below extrema
print_warnings = False          # Debugging info
print_padding = 30              # Print customization
tt = 5                          # Max negative log likelihood that an example can contribute to the total log likelihood
bound_likelihood = False        # Whether we bound the likelihood by tt
max_iter = 150                  # Max number of iterations of a single optimization run

program_complexity_loss = 0.1   # adds L1 loss ( num_parameters * program_complexity_loss )
alpha_loss_lower = 0.2          # adds L2 loss ( 1/alpha^2 * alpha_loss_lower )
alpha_loss_upper = 0.01         # adds L2 loss ( alpha^2 * alpha_loss_upper )
x_0_loss = 0                    # adds L1 loss ( x_0 * x_0_loss )

max_examples_yes = 50;         # Total number of examples to optimize over when transition is satisfied
max_examples_no = 200;         # Total number of examples to optimize over when a transition is not satisfied

# opt_method = optimization method
# 0: local (BFGS)
# 1: local (L-BFGS-B)
# 2: basin hopping
# 3: dual annealing
# 4: DIRECT

def bound_ll(ll): # Doesn't need to be differentiable because it's not used in gradient descent, only in the final loss calculation
    # if ll<-tt-.5:
    #     ll=-tt
    # elif ll<-tt+.5:
    #     ll=.5*math.pow((tt+.5+ll),2)-tt
    # return ll
    return max(ll, -tt)

# -------- objective function --------------------
def log_loss(x, E_k, y_j, clauses, print_res=False, bounded=bound_likelihood):
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
                log_likelihood=bound_ll(log_likelihood)
            log_loss -= log_likelihood
        else:       # Unsatisfied transition
            log_not = sp.logsumexp([0, log_likelihood], b=[1, -1])
            if bounded:
                log_not=bound_ll(log_not)
            log_loss -= log_not
    
    if print_res:
        print("overall LL "+str(log_loss))

    # Calculate parameter loss: based on prior
    param_loss = len(x) * program_complexity_loss
    for inv in alpha_inv:
        param_loss += inv * inv * alpha_loss_lower
        param_loss += 1.0/inv * 1.0/inv * alpha_loss_upper
    for val in x_0:
        param_loss += abs(val) * x_0_loss

    return log_loss / len(y_j) + param_loss



# ------- helper functions ----------------------
def extension(l, r): # list range
    return (max(l, r) - min(l, r)) * bounds_extension

def debug(str):
    if print_debug:
        print(str)

def print_with_padding(label, value):
    debug((label+" ").ljust(print_padding, "-")+" "+str(value))

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

    if(opt_method == 0):        # BFGS (Gradient descent) - local optimization
        res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='BFGS', options={'maxiter': max_iter, 'disp': False})
    elif(opt_method == 1):      # L-BFGS-B (Gradient descent) - local optimization
        if bound_alpha:
            res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='L-BFGS-B', options={'maxiter': max_iter, 'disp': False, 'eps': 1e-2}, bounds=bounds, tol=1e-16)
        else:
            res = optimize.minimize(log_loss, init, args=extra_args, 
                                method='L-BFGS-B', options={'maxiter': max_iter, 'disp': False})
    elif(opt_method == 2):      # Basin hopping - global optimization
        res = optimize.basinhopping(log_loss, init,
                                niter=max_iter, T=100.0,
                                minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, 
                                take_step=step_obj, callback=print_fun)
    elif(opt_method == 3):      # Dual annealing - global optimization
        res = optimize.dual_annealing(log_loss, bounds, x0=init, 
                                        maxiter=max_iter, initial_temp=50000, 
                                        visit=3.0, accept=-5, 
                                        minimizer_kwargs=minimizer_kwargs)
    elif(opt_method == 4):      # DIRECT - global optimization
        res = optimize.direct(log_loss, bounds_arr, args=extra_args, maxiter=max_iter)
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

    if(not print_warnings):
        warnings.filterwarnings('ignore')

    # Sampling: take a subset of the given expressions to optimize over
    count_yes = 0
    count_no = 0
    E_k_subset = []
    y_j_subset = []
    for _ in range(0, len(E_k)):
        E_k_subset.append([])
    
    for i in range(0, len(y_j)):
        if y_j[i] and count_yes < max_examples_yes:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            count_yes += 1
        if not y_j[i] and count_no < max_examples_no:
            for ind in range(0, len(E_k)):
                E_k_subset[ind].append(E_k[ind][i])
            y_j_subset.append(y_j[i])
            count_no += 1

    # ---------- Bounds --------------------
    # Custom step function
    step_obj = TakeStep()

    # Bounds on 1/alpha : defined by min_alpha
    alpha_bounds = [(-1.0/min_alpha, 1.0/min_alpha) for expression in E_k_subset]

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

    for iter in range(max(1, initial_values)):

        # Initialization of x_0

        if initial_values == 0 or iter == 0:
            x_0_init = np.zeros(len(E_k_subset))                                               # Initialize to 0s
        elif initial_values == 1 or iter == 1:
            x_0_init = [sum(expression)/len(expression) for expression in E_k_subset]          # Initialize to the average
        elif initial_values > 1:
            x_0_init = [np.random.uniform(bound[0], bound[1]) for bound in x_0_init_bounds]  # Initialize randomly 

        for signs in range(pow(2, len(E_k_subset))): # iterate over possible signs for alpha
            
            # Initialization of alpha
            alpha_init = []
            for i in range(len(E_k_subset)):
                alpha_init.append(1.0/initial_alpha if (signs & (1 << i)) else -1.0/initial_alpha)

            if not enumerateSigns: # Initialize to 0 (don't iterate over signs)
                alpha_init = np.zeros(len(E_k_subset))
            
            init = np.concatenate((alpha_init, x_0_init))

            # Calling the optimizer
            input.append(init)
            
            if not enumerateSigns:
                break
        
        if initial_values <= 1:
            break
    
    print_with_padding("Initial values", "|")
    debug(input)
    
    # Run optimizer in parallel
    partial_optimizer = functools.partial(run_optimizer_from_initial, E_k_subset, y_j_subset, clauses, bounds, bounds_arr, bounds_obj, step_obj)
    with multiprocessing.Pool(num_cores) as p:
        output = p.map(partial_optimizer, input)
    
    bestRes = min(output, key=lambda i: i.fun)
    

    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value - training set", bestRes.fun)
    bestRes.fun = log_loss(bestRes.x, E_k, y_j, clauses, False, True)
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