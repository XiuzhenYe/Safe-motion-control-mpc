import numpy as np
import cvxpy as cp
from Helper_FindClosestObstacle import find_closest_obstacle

from Config import CFG
dt = CFG.dt
T  = CFG.T
steps = int(T / dt)
N = CFG.N  # MPC horizon steps
# Goal
qg = CFG.qg
# Obstacles: (c, r)
obstacles = CFG.obstacles
# CBF parameters
KP = CFG.KP
alpha = CFG.alpha

# MPC weights
w_q = CFG.w_q     # position-to-goal weight
w_u   = CFG.w_u    # control effort weight

# Constraints
u_max = CFG.u_max     # acceleration bound (m/s^2)
v_max = CFG.v_max     # velocity bound

# Initial condition
q = CFG.q
v = CFG.v

# CBF value storage
h_active = np.zeros((steps,1))

def solve_cbf_mpc_scp(q0, v0, qg, scp_iters=3, N=10):
    """
    Tracks goal position and safe velocity reference.
    """
    ## ================== Build initial reference rollout ===============
    ## ================== to make CBF constraint convex =================
    q_ref = np.zeros((2, N+1))
    v_ref = np.zeros((2, N+1))
    q_ref[:, 0], v_ref[:, 0] = q0, v0
    for k in range(N):
        q_ref[:, k+1] = q_ref[:, k] + dt*v_ref[:,k]
        v_ref[:, k+1] = v_ref[:, k] 

    ## ================== CVX for MPC ===================================
    w_slack = 1e4 # a big penalty to discourage violation
    s = cp.Variable(N, nonneg=True) # slac variable for CBF constraint at time k
    for it in range(scp_iters):  # use current q_ref to build convex constraints to solve convex MPC, update q_ref solution, repeat scp_iters times
        q_var = cp.Variable((2, N+1)) # q
        v_var = cp.Variable((2, N+1)) # dot(q)
        u_var = cp.Variable((2, N)) # dot(dot(q)), that is u is acceleration as an input
        # Initial condition
        cost, constr = 0, []
        constr += [q_var[:, 0] == q0]
        constr += [v_var[:, 0] == v0]

        # Dynamics + constraints + cost
        for k in range(N):
            # dynamics
            constr += [q_var[:, k+1] == q_var[:, k] + dt * v_var[:, k]]
            constr += [v_var[:, k+1] == v_var[:, k] + dt * u_var[:, k]]

            ## ===== constraints ===========================================================
            constr += [cp.norm_inf(u_var[:, k]) <= u_max] # bound for acceleration
            constr += [cp.norm_inf(v_var[:, k]) <= v_max] # bound for speed
            # ------ CBF safe velocity -----------------------------------------------------
            q_obs, _, r_obs = find_closest_obstacle(q_ref[:, k], obstacles)
            r_safe = r_obs + 0.1 # 0.1 as a r_buffer to ensure, it adds margin
            n_k = (q_ref[:, k] - q_obs) / (np.linalg.norm(q_ref[:, k] - q_obs) + 1e-6) # normal direction between q_ref and q_obs
            constr += [n_k @ v_var[:, k] >= -alpha*(n_k@(q_var[:, k] - q_obs) - r_safe) - s[k]]
            
            ## ===== cost ===================================================================
            cost += 0.5 * w_u * cp.sum_squares(u_var[:, k])
            cost += 0.5 * w_q * cp.sum_squares(q_var[:, k] -qg)
            cost += 0.5 * w_slack * cp.square(s[k])
        # terminal cost
        cost += 5.0 * w_q * cp.sum_squares(q_var[:, N] - qg)
 
        prob = cp.Problem(cp.Minimize(cost), constr)
        prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)

        if prob.status not in ["optimal", "optimal_inaccurate"]:
            print("Not feasible")
            return None  
        q_ref, v_ref = q_var.value, v_var.value

    return u_var[:, 0].value