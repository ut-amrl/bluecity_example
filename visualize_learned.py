import h5py
import numpy as np
from matplotlib import pyplot as plt
import pickle


# Speedway - groundtruth
with open("data/demo_trajectories_udr.pickle", 'rb') as g:
    GT_speedway = pickle.load(g)

# Speedway - learned
learned_speedway = h5py.File('data/learned_x_trajectories_udr.jld2', 'r')
learned_MAIRL_x_trajectories_speedway = learned_speedway['x_trajectories'] 


# Simulated - groundtruth
GT = h5py.File('data/GT.jld2', 'r')
GT_simulated = GT['x_trajectories'] 

# Simulated - learned
learned = h5py.File('data/Learned.jld2', 'r')
learned_MAIRL_x_trajectories_simulated = learned['x_trajectories'] 

# Behavior Cloning - learned
with open("data/learned_BC.pkl", 'rb') as f:
    learned_BC_x_trajectories_simulated = pickle.load(f)
with open("data/learned_BC_speedway.pkl", 'rb') as h:
    learned_BC_x_trajectories_speedway = pickle.load(h)

fig_dem, ax_dem = plt.subplots(1, 1)
ax_dem.axis("equal")

# Speedway
ax_dem.set_xlim(-15, 0)
ax_dem.set_ylim(-2, 12)

# Simulated
# ax_dem.set_xlim(-4, 4)
# ax_dem.set_ylim(-4, 4)

# SPEEDWAY

for trajectory in GT_speedway['x']:
    ax_dem.plot(trajectory[:, 0], trajectory[:, 1], alpha=0.2, color="red")
    ax_dem.plot(trajectory[:, 4], trajectory[:, 5], alpha=0.2, color="blue")
    ax_dem.plot(trajectory[:, 8], trajectory[:, 9], alpha=0.2, color="green")

i = 0 #3,6 for RLD, 7 for udr, none for rlu, 6 for udl
for trajectory in learned_MAIRL_x_trajectories_speedway:
    if i==7:    
        trajectory = learned_speedway[trajectory]
        trajectory = np.array(trajectory)
        trajectory = trajectory.T
        # CUT=trajectory.shape[0]
        CUT=20
        ax_dem.plot(trajectory[:CUT,0], trajectory[:CUT, 1], alpha=1, color="red", linewidth=3, linestyle='--')
        ax_dem.plot(trajectory[:CUT, 4], trajectory[:CUT, 5], alpha=1, color="blue", linewidth=3, linestyle='--')
        ax_dem.plot(trajectory[:CUT, 8], trajectory[:CUT, 9], alpha=1, color="green", linewidth=3, linestyle='--')
        
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent1"][0][:CUT], learned_BC_x_trajectories_speedway["agent1"][1][:CUT], alpha=1, color="red", linewidth=3, linestyle='--')
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent2"][0][:CUT], learned_BC_x_trajectories_speedway["agent2"][1][:CUT], alpha=1, color="blue", linewidth=3, linestyle='--')
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent3"][0][:CUT], learned_BC_x_trajectories_speedway["agent3"][1][:CUT], alpha=1, color="green", linewidth=3, linestyle='--')
        plt.show()
    i += 1

# SIMULATED

# for trajectory in GT_simulated:
#     trajectory = GT[trajectory]
#     trajectory = np.array(trajectory)
#     trajectory = trajectory.T
#     ax_dem.plot(trajectory[:, 0], trajectory[:, 1], alpha=.3, color="red")
#     ax_dem.plot(trajectory[:, 4], trajectory[:, 5], alpha=.3, color="blue")
#     ax_dem.plot(trajectory[:, 8], trajectory[:, 9], alpha=.3, color="green")


# for trajectory in learned_MAIRL_x_trajectories_simulated:
#     trajectory = learned[trajectory]
#     trajectory = np.array(trajectory)
#     trajectory = trajectory.T
#     # ax_dem.plot(trajectory[:, 0], trajectory[:, 1], alpha=1, color="red", linewidth=3, linestyle='--')
#     # ax_dem.plot(trajectory[:, 4], trajectory[:, 5], alpha=1, color="blue",linewidth=3, linestyle='--')
#     # ax_dem.plot(trajectory[:, 8], trajectory[:, 9], alpha=1, color="green",linewidth=3, linestyle='--')

#     ax_dem.plot(learned_BC_x_trajectories_simulated["agent1"][0], learned_BC_x_trajectories_simulated["agent1"][1], alpha=1, color="red", linewidth=3, linestyle='--')
#     ax_dem.plot(learned_BC_x_trajectories_simulated["agent2"][0], learned_BC_x_trajectories_simulated["agent2"][1], alpha=1, color="blue", linewidth=3, linestyle='--')
#     ax_dem.plot(learned_BC_x_trajectories_simulated["agent3"][0], learned_BC_x_trajectories_simulated["agent3"][1], alpha=1, color="green", linewidth=3, linestyle='--')
#     plt.show()  

# file.close()
