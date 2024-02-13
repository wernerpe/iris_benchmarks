from pydrake.all import VPolytope, MathematicalProgram, Solve
import numpy as np

def get_AABB_limits(vp : VPolytope, tol = 1e-3): 
    dim = vp.ambient_dimension()
    max_limits = []
    min_limits = []

    for idx in range(dim):
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = x[idx]
        aabbprog.AddCost(cost)
        vp.AddPointInSetConstraints(aabbprog, x)
        result = Solve(aabbprog)
        min_limits.append(result.get_optimal_cost() -tol)
        aabbprog = MathematicalProgram()
        x = aabbprog.NewContinuousVariables(dim, 'x')
        cost = -x[idx]
        aabbprog.AddCost(cost)
        vp.AddPointInSetConstraints(aabbprog, x)
        result = Solve(aabbprog)
        max_limits.append(-result.get_optimal_cost() + tol)
    return min_limits, max_limits

def get_AABB_cvxhull(vertices):
    vps = VPolytope(vertices.T).GetMinimalRepresentation()
    min, max = get_AABB_limits(vps)    
    return np.array(min), np.array(max)
