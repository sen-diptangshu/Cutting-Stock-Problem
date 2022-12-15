# author - D Sen - 29 Nov, 2022
# For ISYE6661 - Linear Optimization
# Implementation of the Stock-cutting problem
# Any parameters (W, w, n, bar_J can be changed)

import gurobipy as gp
from gurobipy import *


# parameters
W = 100                    # width of large rolls
w = [25, 35, 50]           # width of target pieces
n = [20, 30, 40]           # demand of each target piece
bar_J = [[4, 0, 0], [0, 1, 1]]    # initial set of patterns to solve continuous relaxation of primal

m = len(w)
bounds = [0]*m
for i in range(0, m):
    bounds[i] = 1+math.floor(W/w[i])


# Functions
# function for swapping list entries
def swap(a, b, A):
    temp = A[a]
    A[a] = A[b]
    A[b] = temp
    return A


# function for checking if optimal dual solution of restricted problem satisfies condition for particular pattern
def check(index, vector):
    if sum(vector[j]*res_J[index][j] for j in range(0, m)) <= 1:
        return 1
    else:
        return 0


# function to generate a m-tuple which could be a pattern
def generate_pattern(u, e_list):
    sz = len(e_list)
    new_list = []
    for i in range(0, u):
        for j in range(0, sz):
            temp = e_list[j].copy()
            temp.append(i)
            new_list.append(temp)
    return new_list

# End - Function Definitions


# generating set of all patterns J
Big_J = []
for i0 in range(0, bounds[0]):
    Big_J.append([i0])

for k in range(1, m):        # recursion step to generate all m-tuples
    new = generate_pattern(bounds[k], Big_J)
    Big_J = new

Pattern_J = []               # master set of all patterns
for ind in range(0, len(Big_J)):
    if 0 < sum(Big_J[ind][j]*w[j] for j in range(0, m)) <= W:
        Pattern_J.append(Big_J[ind])


# restricted set of patterns
size = len(bar_J)
r = range(0, m)
res_J = Pattern_J.copy()
for i in range(0, size):
    res_J.remove(bar_J[i])         # remove the patterns already in bar_J. Because they are already considered.
N = len(res_J)
run_flag = 1
itr = 1

while run_flag == 1:
    print("\n itr ", itr)

    with gp.Env(empty=True) as env:        # solving the restricted dual problem
        env.setParam('OutputFlag', 0)
        env.start()
        with gp.Model(env=env) as model:

            # variables
            p = model.addVars(m, lb=0, vtype=GRB.CONTINUOUS, name="pi")

            obj = sum(n[j]*p[j] for j in r)

            # constraints
            for k in range(0, size):
                model.addConstr(sum(bar_J[k][j]*p[j] for j in r) <= 1)

            model.setObjective(obj, GRB.MAXIMIZE)
            model.optimize()

            h = model.getVars()
            v = model.getAttr("Pi")          # dual of the dual is the primal

            p_optimal = []
            for j in r:
                p_optimal.append(p[j].x)

    print("bar_p ", p_optimal)

    tick = 0
    for i in range(0, N):
        tick = check(i, p_optimal)    # checking constraint by constraint if the inequality holds
        if tick == 0:                 # if any constraint violates, it is added back to bar_J
            print("J_bar_old ", bar_J)
            bar_J.append(res_J[i])
            temp_pattern = res_J[i]
            res_J.remove(temp_pattern)
            N = N-1
            size = size + 1
            itr = itr+1
            print("J_bar_new ", bar_J)
            break

    if tick == 1:                     # stopping condition - when all constraints are satisfied
        run_flag = 0


# printing solution
print("J_bar_final ", bar_J)
print("\n")
print("Number of rolls of each pattern - ", v)


