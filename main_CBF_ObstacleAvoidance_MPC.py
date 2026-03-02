import numpy as np
import matplotlib.pyplot as plt
from Helper_CBF_MPC import solve_cbf_mpc_scp
from Helper_FindClosestObstacle import find_closest_obstacle
from matplotlib.collections import LineCollection
# ==============================
# Problem setup
# ==============================
from Config import CFG
dt = CFG.dt
T  = CFG.T
steps = int(T / dt)
N = CFG.N  # MPC horizon steps

# Goal
qg = CFG.qg
obstacles = CFG.obstacles

# Initial condition
q = CFG.q
v = CFG.v

# CBF value storage
h_active = np.zeros((steps,1))



# ==============================
# Simulation loop
# ==============================
traj = []
for k in range(steps):
    # MPC chooses acceleration to track v_safe
    u = solve_cbf_mpc_scp(q, v, qg, scp_iters=3, N = N)
    if u is None:
        u = np.zeros(2) 

    # Apply dynamics
    v = v + dt * u
    q = q + dt * v

    # CBF 
    _, h_active[k], _ = find_closest_obstacle(q, obstacles)

    # traj 
    traj.append(q.copy())
 

traj = np.array(traj)
 
# ==============================
# Plot: Trajectory + obstacles
# ==============================
plt.figure()
points = traj.reshape(-1,1,2)
segments = np.concatenate([points[:-1], points[1:]], axis=1)
t = np.linspace(0,int(T),len(segments))
lc = LineCollection(segments, cmap='viridis', linewidth=2)
lc.set_array(t)
ax = plt.gca()
ax.add_collection(lc)
plt.scatter([qg[0]], [qg[1]], marker="*", s=150, label="goal", color='b')
plt.scatter([0], [0], marker="o", s=90, label="start", color = 'r')
for (center, r) in obstacles:
    circ = plt.Circle(center, r, fill=False)
    ax.add_patch(circ)
    plt.scatter(center[0], center[1], marker="x", color='k')
cbar = plt.colorbar(lc)
cbar.set_label("Time (s)")
plt.axis("equal")
plt.legend()
plt.savefig("CBF MPC trajectory.png")
plt.xlabel("$q_1$")
plt.ylabel("$q$")
plt.legend()
plt.show()

# ==============================
# Plot: CBF value over time
# ==============================
plt.figure()
plt.plot(np.linspace(0, int(T), steps), h_active)
plt.axhline(0.0, linewidth=1.0)
plt.xlabel("time, $t$(s)")
plt.ylabel("CBF, $h$(m))")
plt.savefig("CBF safety margin over time in MPC.png")
plt.show()