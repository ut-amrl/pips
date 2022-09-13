import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from scipy import optimize
from scipy import special as sp

import math
import warnings
import time
import json

# TODO
# how to weight all examples equally, so as not to bias towards the ones with high / extreme values
# or just control the range of examples during data collection
# deal with integer overflow in loss function (results in nan loss)
# use log-likelihoods as soon as possible, and use log operations when possible to eliminate integer overflow
# optimization methods are highly dependent on initial guess
# for most applications starting at 0 as the center of the log function is probably best?
# or is it cheating because that's the only constant in this specific simulation 
# calculate ideal log-likelihood for each ASP transition
# bounds for basin hopping and BFGS is broken (maybe my fault)
# for some reason ACC->CON works when I do it manually (with local BFGS starting at x_0 = 0) but not when PIPS
# maybe has to do with choosing the samples? that is the only difference
# in this manual test i take all the transitions compared to PIPS only taking from the window
# maybe just expand window idk

# ------- Parameters -----------------------------
opt_method = 0
enumerateSigns = True
print_debug = False

start_x_0_at_0 = True
max_spread = 10.0
min_spread = 0.1
bounds_extension = 0.1
print_warnings = False
print_padding = 30

# optimization method
# 0: local
# 1: basin hopping
# 2: dual annealing
# 3: DIRECT

# ------- Variable declarations -----------------------------
E_k = []
y_j = []
clauses = []

# -------- objective function --------------------
def log_loss(x):
    alpha = x[: len(x)//2] # values of alpha (slope) for each conditional structure
    x_0 = x[len(x)//2 :] # center of logistic function for each conditional structure

    log_loss = 0
    for i in range(len(y_j)):
        log_likelihood = - sp.logsumexp([0, - alpha[0] * (E_k[0][i] - x_0[0])])

        for j in range(len(clauses)):
            log_likelihood_j = - sp.logsumexp([0, - alpha[j+1] * (E_k[j+1][i] - x_0[j+1])])

            if clauses[j] == 0:
                log_likelihood += log_likelihood_j
            if clauses[j] == 1:
                log_likelihood = sp.logsumexp([log_likelihood, log_likelihood_j, log_likelihood+log_likelihood_j], b=[1, 1, -1])
            
        if y_j[i]:
            log_loss -= log_likelihood
        else:
            log_loss -= sp.logsumexp([0, log_likelihood], b=[1, -1])
    return log_loss

# ------- helper functions ----------------------
def lr(l): # list range
    return max(l)-min(l)

def debug(str):
    if print_debug:
        print(str)

def print_with_padding(label, value):
    debug((label+" ").ljust(print_padding, "-")+" "+str(value))

# --------- optimizer -----------------------------
def run_optimizer(E_k_loc, y_j_loc, clauses_loc):
    global E_k, y_j, clauses
    E_k = E_k_loc
    y_j = y_j_loc
    clauses = clauses_loc

    if(not print_warnings):
        warnings.filterwarnings('ignore')

    minimizer_kwargs = {"method": "BFGS"}
    rng = np.random.default_rng()


    class Bounds:
        def __init__(self, xmin, xmax):
            self.xmax = np.array(xmax)
            self.xmin = np.array(xmin)
        def __call__(self, **kwargs):
            x = kwargs["x_new"]
            tmax = bool(np.all(x <= self.xmax))
            tmin = bool(np.all(x >= self.xmin))
            return tmax and tmin
    
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
            # x += (np.random.randint(2, size=x.shape)-0.5)*2
            return x

    step_obj = TakeStep()

    # ---------- Callback ------------------------
    def print_fun(x, f, accepted):
        # debug("at minimum %.4f accepted %d" % (f, int(accepted)))
        # debug("with parameters: ")
        # debug(x)
        return

    bestRes = -1
    for signs in range(pow(2, len(E_k))): # iterate over possible signs (corresponds to < and >)
        
        # ---------- Initialization ------------
        alpha_init = []
        for i in range(len(E_k)):
            alpha_init.append(1 if (signs & (1 << i)) else -1)

        if not enumerateSigns:
            alpha_init = np.zeros(len(E_k))
        
        x_0_init = [sum(expression)/len(expression) for expression in E_k]
        if start_x_0_at_0:
            x_0_init = [0.0, 0.0]
        init = np.concatenate((alpha_init, x_0_init))
        print_with_padding("Initial values", "|")
        debug(init)

        # ---------- Bounds --------------------
        alpha_bounds = []
        for i in range(len(E_k)):
            alpha_bounds.append((min_spread, max_spread) if (signs & (1 << i)) else (-max_spread, -min_spread))

        if not enumerateSigns:
            alpha_bounds = [(-max_spread, max_spread) for expression in E_k]

        x_0_bounds = [(min(expression)-lr(expression)*bounds_extension, max(expression)+0.000001+lr(expression)*bounds_extension) for expression in E_k]
        bounds = np.concatenate((alpha_bounds, x_0_bounds))
        bounds_lower = [b[0] for b in bounds]
        bounds_upper = [b[1] for b in bounds]
        print_with_padding("Bounds", "|")
        debug(bounds)
        bounds_obj = Bounds(bounds_lower, bounds_upper)
        bounds_arr = optimize.Bounds(bounds_lower, bounds_upper)

        # -------- Optimization ----------------------

        if(opt_method == 0):
            res = optimize.minimize(log_loss, init,
                                    method='BFGS', options={'disp': print_debug, 'maxiter': 1000})
        elif(opt_method == 1):
            res = optimize.basinhopping(log_loss, init,
                                    niter=100, T=40000.0,
                                    minimizer_kwargs=minimizer_kwargs, accept_test=bounds_obj, 
                                    take_step=step_obj, callback=print_fun, seed=rng)
        elif(opt_method == 2):
            res = optimize.dual_annealing(log_loss, bounds, x0=init, 
                                            maxiter=50, initial_temp=50000, 
                                            visit=3.0, accept=-5, 
                                            minimizer_kwargs=minimizer_kwargs)
        elif(opt_method == 3):
            res = optimize.direct(log_loss, bounds_arr, 
                                    maxiter=10000)
        else:
            sys.exit("Please use a valid optimization method")

        print_with_padding("Optimal parameters", "|")
        debug(res.x)
        print_with_padding("Num iterations", res.nfev)
        print_with_padding("Minimum value", res.fun)
        debug("")

        if bestRes == -1 or res.fun < bestRes.fun:
            bestRes = res
        
        if not enumerateSigns:
            break
    
    print_with_padding("Final parameters", "|")
    debug(bestRes.x)
    print_with_padding("Minimum value", bestRes.fun)

    return (bestRes.fun, list(bestRes.x))



# --------------------- testing -------------------------------------------

def dist_traveled(v, dec):
    return - v * v / (2 * dec)

def main():

    f = open('examples/emdips/out/data.json')
    data = json.load(f)
    E_k_test = []
    y_j_test = []
    clauses_test = [0]
    arr_a = []
    arr_b = []
    for row in data:
        if(row['start']['value'] == 'ACC'):
            y_j_test.append(int(row['output']['value'] == 'CON'))
            arr_a.append(row['v']['value'] - row['vMax']['value'])
            arr_b.append(dist_traveled(row['v']['value'], row['decMax']['value']) + row['x']['value'] - row['target']['value'])

    E_k_test = [arr_a, arr_b]

    f.close()

    # run_optimizer(E_k_test, y_j_test, clauses_test)

    global E_k, y_j, clauses
    E_k = E_k_test
    y_j = y_j_test
    clauses = clauses_test

    print("supposed")
    print(log_loss([1, -1, 0.1, 0]))
    print(log_loss([1, -1, 0, 0]))
    print(log_loss([2, -2, 0, 0]))
    print(log_loss([10, -10, 0, 0]))

if __name__ == "__main__":
    main()
