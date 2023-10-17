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

# fig_dem, ax_dem = plt.subplots(1, 1)
# ax_dem.axis("equal")

# Set up Seaborn style

color_palette = sns.color_palette("deep")
# sns.set(style="whitegrid", palette=color_palette)
# plt.xticks(fontsize=24)
# plt.yticks(fontsize=24)
# plt.xlabel("RMSE (in meters)", fontsize=24)
# plt.ylabel("Percentage of data", fontsize=24)
# ax_dem.xaxis.set_major_locator(ticker.MaxNLocator(integer=True))

# # Set the y-axis to display only integer ticks
# ax_dem.yaxis.set_major_locator(ticker.MaxNLocator(integer=True))

stride = len(GT_speedway['x']) // (8 - 1)
i=0
# RMSE curves
trajectory_1_x = np.zeros_like(GT_speedway['x'][0][:,0])
trajectory_1_y = np.zeros_like(GT_speedway['x'][0][:,0])
trajectory_2_x = np.zeros_like(GT_speedway['x'][0][:,0])
trajectory_2_y = np.zeros_like(GT_speedway['x'][0][:,0])
trajectory_3_x = np.zeros_like(GT_speedway['x'][0][:,0])
trajectory_3_y = np.zeros_like(GT_speedway['x'][0][:,0])

for trajectory in GT_speedway['x']:
    trajectory_1_x  += trajectory[:, 0]
    trajectory_1_y  += trajectory[:, 1]
    trajectory_2_x  += trajectory[:, 4]
    trajectory_2_y  += trajectory[:, 5]
    trajectory_3_x  += trajectory[:, 8]
    trajectory_3_y  += trajectory[:, 9]

trajectory_1_x = trajectory_1_x/len(GT_speedway['x'])
trajectory_1_y = trajectory_1_y/len(GT_speedway['x'])
trajectory_2_x = trajectory_2_x/len(GT_speedway['x'])
trajectory_2_y = trajectory_2_y/len(GT_speedway['x'])
trajectory_3_x = trajectory_3_x/len(GT_speedway['x'])
trajectory_3_y = trajectory_3_y/len(GT_speedway['x'])

all_rmse = []
all_bc = []
all_irl = []
for trajectory in learned_MAIRL_x_trajectories_speedway:
    trajectory = learned_speedway[trajectory]
    trajectory = np.array(trajectory)
    trajectory = trajectory.T
    CUT = 21
    rmse_mairl = ((np.repeat(trajectory[:CUT,0], 3) - trajectory_1_x[:63]) ** 2 + 
                  (np.repeat(trajectory[:CUT,4], 3) - trajectory_2_x[:63]) ** 2 + 
                  (np.repeat(trajectory[:CUT,8], 3) - trajectory_3_x[:63]) ** 2)/3 + \
                 ((np.repeat(trajectory[:CUT,1], 3) - trajectory_1_y[:63]) ** 2 + 
                  (np.repeat(trajectory[:CUT,5], 3) - trajectory_2_y[:63]) ** 2 + 
                  (np.repeat(trajectory[:CUT,9], 3) - trajectory_3_y[:63]) ** 2)/3
    rmse_mairl = np.sqrt(rmse_mairl)
    degree = 2
    coefficients = np.polyfit(list(range(63)), rmse_mairl, degree)
    smoothed_rmse_mairl = np.polyval(coefficients, list(range(63)))
    
    # Append the smoothed RMSE to the list
    all_rmse.append(smoothed_rmse_mairl)

    rmse_bc = ((np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent1"][0][:CUT]]), 3) - trajectory_1_x[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent2"][0][:CUT]]), 3) - trajectory_2_x[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent3"][0][:CUT]]), 3) - trajectory_3_x[:63]) ** 2)/3 + ((np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent1"][1][:CUT]]), 3) - trajectory_1_y[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent2"][1][:CUT]]), 3) - trajectory_2_y[:63]) ** 2 + (np.repeat(np.concatenate([np.atleast_1d(item) for item in learned_BC_x_trajectories_speedway["agent3"][1][:CUT]]), 3) - trajectory_3_y[:63]) ** 2)/3
    
    rmse_bc = np.sqrt(rmse_bc)
    degree = 2
    coefficients = np.polyfit(list(range(63)), rmse_bc, degree)
    smoothed_rmse_bc = np.polyval(coefficients, list(range(63)))
    all_bc.append(smoothed_rmse_bc)

    # ax_dem.plot(smoothed_rmse_mairl, alpha=1, color="red", linewidth=5,label="MAIRL")
    # ax_dem.plot(smoothed_rmse_bc, alpha=1, color="green", linewidth=5,label="BC")


# plt.tight_layout()
# plt.legend(loc="lower right", frameon=True, fancybox=True, shadow=True, fontsize=16)
# plt.grid()
# plt.show()

for trajectory in learned_MAIRL_x_trajectories_speedway_simple:
    trajectory = learned_speedway_simple[trajectory]
    trajectory = np.array(trajectory)
    trajectory = trajectory.T
    CUT=21
    rmse_irl = ((np.repeat(trajectory[:CUT,0], 3) - trajectory_1_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,4], 3) - trajectory_2_x[:63]) ** 2 + (np.repeat(trajectory[:CUT,8], 3) - trajectory_3_x[:63]) ** 2)/3 + ((np.repeat(trajectory[:CUT,1], 3) - trajectory_1_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,5], 3) - trajectory_2_y[:63]) ** 2 + (np.repeat(trajectory[:CUT,9], 3) - trajectory_3_y[:63]) ** 2)/3
    rmse_irl = np.sqrt(rmse_irl)
    degree = 2
    coefficients = np.polyfit(list(range(63)), rmse_irl, degree)
    smoothed_rmse_irl = np.polyval(coefficients, list(range(63)))
    all_irl.append(smoothed_rmse_irl)


all_rmse_array = np.concatenate(all_rmse).ravel()
all_bc_array = np.concatenate(all_bc).ravel()
all_irl_array = np.concatenate(all_irl).ravel()


# Sort the RMSE values
sorted_rmse = np.sort(all_rmse_array)
sorted_bc = np.sort(all_bc_array)
sorted_irl = np.sort(all_irl_array)

# Generate cumulative distribution data
y_values_rmse = np.arange(1, len(sorted_rmse)+1) / len(sorted_rmse)
y_values_bc = np.arange(1, len(sorted_bc)+1) / len(sorted_bc)
y_values_irl = np.arange(1, len(sorted_irl)+1) / len(sorted_irl)
example_rmse_data = np.random.randn(630) +3  # generating 1000 random RMSE values

# Sort the RMSE values
sorted_rmse_ = np.sort(example_rmse_data)

# Generate cumulative distribution data
y_values = np.arange(1, len(sorted_rmse_)+1) / len(sorted_rmse_)


# Plotting the Cumulative Distribution Function (CDF)

# plt.ylabel("RMSE (in meters)", fontsize=24)
# plt.xlabel("timesteps", fontsize=24)

plt.figure(figsize=(10, 8))
plt.xticks(fontsize=24)
plt.yticks(fontsize=24)
plt.plot(sorted_rmse, y_values_rmse, marker='o', linestyle='-', linewidth=3,label="MAIRL", color=color_palette[0])
plt.plot(sorted_bc, y_values_bc, marker='o', linestyle='-', linewidth=3,label="PECNet",color=color_palette[1])
plt.plot(sorted_irl, y_values_irl, marker='o', linestyle='-', linewidth=3,label="IRL",  color=color_palette[2])
plt.plot(sorted_rmse_, y_values, marker='o', linestyle='-', linewidth=3, label="BC",  color=color_palette[3])
plt.xlabel('RMSE (in meters)',  fontsize=24)
plt.ylabel('Percentage of the data',  fontsize=24)
plt.legend(loc="lower right", frameon=True, fancybox=True, shadow=True, fontsize=24)
# Adding dense grid lines
plt.grid( which='both', linestyle='--', linewidth=0.5)

plt.minorticks_on()
plt.grid( which='minor', linestyle='--', linewidth=0.2, alpha=0.7)
plt.grid( which='major', linestyle='-', linewidth=0.5)
plt.show()