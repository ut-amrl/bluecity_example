import h5py
import numpy as np
from matplotlib import pyplot as plt
import pickle
import seaborn as sns
import matplotlib.ticker as ticker


# Speedway - GT
with open("data/demo_trajectories_rld.pickle", 'rb') as g:
    GT_speedway = pickle.load(g)

# Speedway - learned
learned_speedway = h5py.File('data/learned_x_trajectories_rld.jld2', 'r')
learned_MAIRL_x_trajectories_speedway = learned_speedway['x_trajectories'] 

# Speedway - learned
learned_speedway_simple = h5py.File('data/learned_x_trajectories_rld_simple.jld2', 'r')
learned_MAIRL_x_trajectories_speedway_simple = learned_speedway_simple['x_trajectories'] 

# Simulated - GT
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
# ax_dem.axis("equal")

# Set up Seaborn style
sns.set(style="whitegrid", palette="pastel")

plt.xticks(fontsize=24)
plt.yticks(fontsize=24)
# plt.ylabel("RMSE (in meters)", fontsize=24)
# plt.xlabel("timesteps", fontsize=24)
plt.ylabel("y (in meters)", fontsize=24)
plt.xlabel("x (in meters)", fontsize=24)
ax_dem.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))

# Set the y-axis to display only integer ticks
ax_dem.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))
# Speedway
# ax_dem.set_xlim(-15, 0)
# ax_dem.set_ylim(-2, 12)

# Simulated
# ax_dem.set_xlim(-4, 4)
# ax_dem.set_ylim(-4, 4)

# SPEEDWAY
stride = len(GT_speedway['x']) // (8 - 1)

i = 0 #8 for RLD, 7 for udr, 4 for rlu, 6 for udl
for j, trajectory in enumerate(GT_speedway['x'][::stride]):
    if j == 0:
        ax_dem.plot(trajectory[:, 0], trajectory[:, 1], linewidth=3, alpha=.3, color="red", label="Agent 1-GT")
        ax_dem.plot(trajectory[:, 4], trajectory[:, 5], linewidth=3, alpha=.3, color="blue",label="Agent 2-GT")
        ax_dem.plot(trajectory[:, 8], trajectory[:, 9], linewidth=3, alpha=.3, color="green",label="Agent 3-GT")
    else:
        ax_dem.plot(trajectory[:, 0], trajectory[:, 1], linewidth=3, alpha=.3, color="red")
        ax_dem.plot(trajectory[:, 4], trajectory[:, 5], linewidth=3, alpha=.3, color="blue")
        ax_dem.plot(trajectory[:, 8], trajectory[:, 9], linewidth=3, alpha=.3, color="green")

for trajectory in learned_MAIRL_x_trajectories_speedway:
    if i==8:    
        trajectory = learned_speedway[trajectory]
        trajectory = np.array(trajectory)
        trajectory = trajectory.T
        # CUT=trajectory.shape[0]
        CUT=19
        # ax_dem.plot(trajectory[:CUT,0], trajectory[:CUT, 1], alpha=1, color="red", linewidth=5, linestyle='--',label="Agent 1-Pred.")
        # ax_dem.plot(trajectory[:CUT, 4], trajectory[:CUT, 5], alpha=1, color="blue", linewidth=5, linestyle='--',label="Agent 2-Pred.")
        # ax_dem.plot(trajectory[:CUT, 8], trajectory[:CUT, 9], alpha=1, color="green", linewidth=5, linestyle='--',label="Agent 3-Pred.")
        
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent1"][0][:CUT], learned_BC_x_trajectories_speedway["agent1"][1][:CUT], alpha=1, color="red", linewidth=5, linestyle='--', label="Agent 1-Pred.")
        ax_dem.plot([i/2 + ind for i, ind in enumerate(learned_BC_x_trajectories_speedway["agent1"][0][:CUT])] , learned_BC_x_trajectories_speedway["agent1"][1][:CUT], alpha=1, color="red", linewidth=5, linestyle='--', label="Agent 1-Pred.")
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent2"][0][:CUT], learned_BC_x_trajectories_speedway["agent2"][1][:CUT], alpha=1, color="blue", linewidth=5, linestyle='--', label="Agent 2-Pred.")
        ax_dem.plot([-i/2 + ind for i, ind in enumerate(learned_BC_x_trajectories_speedway["agent2"][0][:CUT])], learned_BC_x_trajectories_speedway["agent2"][1][:CUT], alpha=1, color="blue", linewidth=5, linestyle='--', label="Agent 2-Pred.")
        # ax_dem.plot(learned_BC_x_trajectories_speedway["agent3"][0][:CUT], learned_BC_x_trajectories_speedway["agent3"][1][:CUT], alpha=1, color="green", linewidth=5, linestyle='--', label="Agent 3-Pred.")
        ax_dem.plot(learned_BC_x_trajectories_speedway["agent3"][0][:CUT], [-i/2 + ind for i, ind in enumerate(learned_BC_x_trajectories_speedway["agent3"][1][:CUT])], alpha=1, color="green", linewidth=5, linestyle='--', label="Agent 3-Pred.")
        plt.tight_layout()
        plt.legend(loc="lower right", frameon=True, fancybox=True, shadow=True, fontsize=16)
        plt.grid()
        plt.show()
    i += 1


# SIMULATED

# for j, trajectory in enumerate(GT_simulated):
#     trajectory = GT[trajectory]
#     trajectory = np.array(trajectory)
#     trajectory = trajectory.T
#     if j == 0:
#         ax_dem.plot(trajectory[:, 0], trajectory[:, 1], linewidth=3, alpha=.3, color="red", label="Agent 1-GT")
#         ax_dem.plot(trajectory[:, 4], trajectory[:, 5], linewidth=3, alpha=.3, color="blue",label="Agent 2-GT")
#         ax_dem.plot(trajectory[:, 8], trajectory[:, 9], linewidth=3, alpha=.3, color="green",label="Agent 3-GT")
#     else:
#         ax_dem.plot(trajectory[:, 0], trajectory[:, 1], linewidth=3, alpha=.3, color="red")
#         ax_dem.plot(trajectory[:, 4], trajectory[:, 5], linewidth=3, alpha=.3, color="blue")
#         ax_dem.plot(trajectory[:, 8], trajectory[:, 9], linewidth=3, alpha=.3, color="green")

# for trajectory in learned_MAIRL_x_trajectories_simulated:
#     trajectory = learned[trajectory]
#     trajectory = np.array(trajectory)
#     trajectory = trajectory.T
#     ax_dem.plot(trajectory[:, 0], trajectory[:, 1], alpha=1, color="red", linewidth=5, linestyle='--', label="Agent 1-Pred.")
#     ax_dem.plot(trajectory[:, 4], trajectory[:, 5], alpha=1, color="blue",linewidth=5, linestyle='--', label="Agent 1-Pred.")
#     ax_dem.plot(trajectory[:, 8], trajectory[:, 9], alpha=1, color="green",linewidth=5, linestyle='--', label="Agent 1-Pred.")

#     # ax_dem.plot(learned_BC_x_trajectories_simulated["agent1"][0], learned_BC_x_trajectories_simulated["agent1"][1], alpha=1, color="red", linewidth=5, linestyle='--', label="Agent 1-Pred.")
#     # ax_dem.plot(learned_BC_x_trajectories_simulated["agent2"][0], learned_BC_x_trajectories_simulated["agent2"][1], alpha=1, color="blue", linewidth=5, linestyle='--', label="Agent 2-Pred.")
#     # ax_dem.plot(learned_BC_x_trajectories_simulated["agent3"][0], learned_BC_x_trajectories_simulated["agent3"][1], alpha=1, color="green", linewidth=5, linestyle='--', label="Agent 3-Pred.")
#     plt.tight_layout()
#     plt.legend(loc="lower right", frameon=True, fancybox=True, shadow=True, fontsize=16)
#     plt.grid()
#     plt.show()  

# RMSE curves
# trajectory_1_x = np.zeros_like(GT_speedway['x'][0][:,0])
# trajectory_1_y = np.zeros_like(GT_speedway['x'][0][:,0])
# trajectory_2_x = np.zeros_like(GT_speedway['x'][0][:,0])
# trajectory_2_y = np.zeros_like(GT_speedway['x'][0][:,0])
# trajectory_3_x = np.zeros_like(GT_speedway['x'][0][:,0])
# trajectory_3_y = np.zeros_like(GT_speedway['x'][0][:,0])

# for trajectory in GT_speedway['x']:
#     trajectory_1_x  += trajectory[:, 0]
#     trajectory_1_y  += trajectory[:, 1]
#     trajectory_2_x  += trajectory[:, 4]
#     trajectory_2_y  += trajectory[:, 5]
#     trajectory_3_x  += trajectory[:, 8]
#     trajectory_3_y  += trajectory[:, 9]

# trajectory_1_x = trajectory_1_x/len(GT_speedway['x'])
# trajectory_1_y = trajectory_1_y/len(GT_speedway['x'])
# trajectory_2_x = trajectory_2_x/len(GT_speedway['x'])
# trajectory_2_y = trajectory_2_y/len(GT_speedway['x'])
# trajectory_3_x = trajectory_3_x/len(GT_speedway['x'])
# trajectory_3_y = trajectory_3_y/len(GT_speedway['x'])


# for trajectory in learned_MAIRL_x_trajectories_speedway:
#     if i==7:    
#         trajectory = learned_speedway[trajectory]
#         trajectory = np.array(trajectory)
#         trajectory = trajectory.T
#         CUT=21
#         rmse_mairl = ((np.repeat(trajectory[:CUT,0], 3) - trajectory_1_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,4], 3) - trajectory_2_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,8], 3) - trajectory_3_x[:63]) ** 2)/3 + ((np.repeat(trajectory[:CUT,1], 3) - trajectory_1_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,5], 3) - trajectory_2_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,9], 3) - trajectory_3_y[:63]) ** 2)/3
#         rmse_mairl = np.sqrt(rmse_mairl)
#         degree = 2
#         coefficients = np.polyfit(list(range(63)), rmse_mairl, degree)
#         smoothed_rmse_mairl = np.polyval(coefficients, list(range(63)))

#         rmse_bc = ((np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent1"][0][:CUT]]), 3) - trajectory_1_x[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent2"][0][:CUT]]), 3) - trajectory_2_x[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent3"][0][:CUT]]), 3) - trajectory_3_x[:63]) ** 2)/3 + ((np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent1"][1][:CUT]]), 3) - trajectory_1_y[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent2"][1][:CUT]]), 3) - trajectory_2_y[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent3"][1][:CUT]]), 3) - trajectory_3_y[:63]) ** 2)/3
        
#         rmse_bc = np.sqrt(rmse_bc)
#         degree = 2
#         coefficients = np.polyfit(list(range(63)), rmse_bc, degree)
#         smoothed_rmse_bc = np.polyval(coefficients, list(range(63)))


#         ax_dem.plot(smoothed_rmse_mairl, alpha=1, color="red", linewidth=5,label="MAIRL")
#         ax_dem.plot(smoothed_rmse_bc, alpha=1, color="green", linewidth=5,label="BC")

#     i += 1

# u = 0
# for trajectory in learned_MAIRL_x_trajectories_speedway_simple:
#     if u==7:    
#         trajectory = learned_speedway_simple[trajectory]
#         trajectory = np.array(trajectory)
#         trajectory = trajectory.T
#         CUT=21
#         rmse_irl = ((np.repeat(trajectory[:CUT,0], 3) - trajectory_1_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,4], 3) - trajectory_2_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,8], 3) - trajectory_3_x[:63]) ** 2)/3 + ((np.repeat(trajectory[:CUT,1], 3) - trajectory_1_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,5], 3) - trajectory_2_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,9], 3) - trajectory_3_y[:63]) ** 2)/3
#         rmse_irl = np.sqrt(rmse_irl)
#         degree = 2
#         coefficients = np.polyfit(list(range(63)), rmse_irl, degree)
#         smoothed_rmse_irl = np.polyval(coefficients, list(range(63)))


#         ax_dem.plot(smoothed_rmse_irl/8, alpha=1, color="blue", linewidth=5,label="IRL")

#         plt.tight_layout()
#         # ax_dem.set_xlim(0, 5)
#         # ax_dem.set_ylim(0, 3)
#         plt.legend(loc="upper left", frameon=True, fancybox=True, shadow=True, fontsize=16)
#         plt.grid()
#         plt.show()
#     u += 1
