import numpy as np
import pickle
import pandas as pd
import argparse
import itertools
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


def generate_triplets(list1, list2, list3):
    triplets = list(itertools.product(list1, list2, list3))
    unique_triplets = [list(triplet) for triplet in set(triplets)]
    return unique_triplets

def prepare_interaction_matrix(pickle_file):
    data = None
    # Load the pickle file
    with open(pickle_file, "rb") as f:
        # Load the contents of the file into a Python object
        data = pickle.load(f)
        # data is a nested dictionary mapping object id to a dictionary of the form
        # {'frame': frame_number, 'class': class_type, 'x': x_coordinate, 'y': y_coordinate, 'speed': speed}
    print(data.values())
    if data is None:
        print("No data to process")
        return

    ids = list(data.keys())
    demo_trajectories = []
    u_trajectories = []
    common_frames = set(data[ids[0]]['frames'].keys())
    # print(common_frames)
    for object_ids in ids:
        if len(data[object_ids]['frames']) < 1:
            data.pop(object_ids)
            continue

    for object_ids in list(data.keys()):
        common_frames = set(common_frames)
        if len(common_frames.intersection(set(data[object_ids]['frames'].keys()))) > 1:
            common_frames = common_frames.intersection(set(data[object_ids]['frames'].keys()))
        else:
            data.pop(object_ids)
            continue
        if len(common_frames) == 0:
            print("No common frames between objects")
            return
        common_frames = sorted(list(common_frames))
    with open(f"data_common_frames.pkl", "wb") as f:
        pickle.dump(data, f)
        # np array will be of the format T x (N(4 + N)) where T is the number of common frames and N is the number of objects
        # every (4 + N) columns will be for one object where the first 4 columns are for the object itself and the next N columns is a one hot encoding of 
        # which object it is
        # matrix = np.zeros((len(common_frames), len(object_ids) * (4 + len(object_ids))))
    for obj_id in list(data.keys()):
        x = [frame['x'] for frame in data[obj_id]['frames'].values()]
        y = [frame['y'] for frame in data[obj_id]['frames'].values()]
        objectType = int(data[obj_id]['class'])
        if objectType < 10:  
            line, = plt.plot(x, y, label=obj_id, linestyle='solid')
        elif objectType == 10:  # pedestrian is 10, plot with dotted line
            line, = plt.plot(x, y, label=obj_id, linestyle='dotted')
        else:  # bicycle is 13, plot with dashed line
            line, plt.plot(x, y, label=obj_id, linestyle='dashed')
    # get name of pickle file without the path or extension
    # show the plot
    plt.show()
        # Animation update function
    # Prepare figure and axis
    fig, ax = plt.subplots()
    ax.set_xlim(-40, 60)  # Keep axis range constant
    ax.set_ylim(-60, 40)  # Keep axis range constant
    # Determine maximum frame count
    print(data.values())
    max_frames = max(max(len(obj['frames']) for obj in data.values()), 1)
    # print(max_frames)
    def update(frame):
        ax.clear()
        ax.set_xlim(-40, 40)  # Keep axis range constant
        ax.set_ylim(-60, 40)  # Keep axis range constant
        for obj_id in data:
            x_full = [frame['x'] for frame in data[obj_id]['frames'].values()]
            y_full = [frame['y'] for frame in data[obj_id]['frames'].values()]
            x = x_full[:frame]  # Get x up to current frame
            y = y_full[:frame]  # Get y up to current frame
            objectType = int(data[obj_id]['class'])
            if objectType < 10:
                ax.plot(x, y, linestyle='solid', label=obj_id)
            elif objectType == 10:
                ax.plot(x, y, linestyle='dotted', label=obj_id)
            else:
                ax.plot(x, y, linestyle='dashed', label=obj_id)

    # Creating animation
    ani = FuncAnimation(fig, update, frames=range(1, max_frames + 1))
    ani.save('trajectory_animation.mp4', writer='ffmpeg', fps=15)
    
    matrix = np.zeros((len(common_frames), len(list(data.keys())) * 4))
    umatrix = np.zeros((len(common_frames)-1, len(list(data.keys())) * 2))
    prev_frame_xy = [(0, 0) for i in range(len(list(data.keys())))]
    # populate the matrix
    for frame_idx, frame in enumerate(common_frames):
        for idx, obj_id in enumerate(list(data.keys())):
            # get the object
            obj = data[obj_id]['frames'][frame]
            # get the index of the object in the matrix
            obj_idx = idx * (4)
            # obj_idx = idx * (len(list(data.keys()))+1)
            # populate the matrix
            matrix[frame_idx, obj_idx] = obj['x']
            matrix[frame_idx, obj_idx + 1] = obj['y']
            matrix[frame_idx, obj_idx + 2] = obj['speed']
            # compute theta based on the previous frame
            if frame_idx != 0:
                matrix[frame_idx, obj_idx + 3] = np.arctan2(obj['y'] - prev_frame_xy[idx][1], obj['x'] - prev_frame_xy[idx][0])
                umatrix[frame_idx-1,(idx * 2)] = obj['speed']
                # umatrix[frame_idx-1,(idx * 2)+1] = np.arctan2(obj['y'] - prev_frame_xy[idx][1], obj['x'] - prev_frame_xy[idx][0])
                umatrix[frame_idx-1,(idx * 2)+1] = 0
            # matrix[frame_idx, obj_idx + 4 + idx] = 1
            # update the previous frame for the correct object
            prev_frame_xy[idx] = (obj['x'], obj['y'])
    demo_trajectories.append(matrix)
    u_trajectories.append(umatrix)
    # name of the input pickle file
    pickle_file_name = pickle_file.split("/")[-1].split(".")[0]
    # save the matrix
    # with open('/home/rchandra/Research/MultiAgentIRL/data/demo_trajectories_rld.pickle', 'wb') as file:
    #     pickle.dump({'x': demo_trajectories, 'u': u_trajectories}, file)    

    with open('data/demo_trajectories_large.pickle', 'wb') as fileh:
        pickle.dump({'x': demo_trajectories, 'u': u_trajectories}, fileh)    


if __name__ == "__main__":
    # get input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("pickle_file", help="pickle data file to parse")
    # parser.add_argument("object_ids", nargs="+", help="object ids to visualize")
    args = parser.parse_args()
    if args.pickle_file == None:
        print("Usage: python data_processing.py <pickle_file> <object_ids>")
        sys.exit(1)
    # prepare the interaction matrix
    prepare_interaction_matrix(args.pickle_file)
    # prepare_interaction_matrix("data/trajectories_clean_full_set")










