import numpy as np
import pickle
import pandas as pd
import argparse
import itertools

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
        
    if data is None:
        print("No data to process")
        return
    # ensure the object ids are in the data
    # for obj_id in object_ids:
    #     if obj_id not in data:
    #         print(f"Object {obj_id} not in data")
    #         return
    ids = list(data.keys())

    left = ['169910600', '361039828', '375229858', '976888564', '233484400'] #x_init = -14,7
    right = ['812846356', '366119061', '986149069', '227650253','751640419'] #x_init = 0,7
    up = ['320730597', '233', '690', '507', '604'] #x_init = -9,-2
    down = ['20030848', '770', '54','979','745'] #x_init = -9, 11

    triplets = generate_triplets(right, left, down)

    demo_trajectories = []
    u_trajectories = []
    common_frames = set(data[ids[0]]['frames'].keys())
    for object_ids in ids:
        # get common frames between all objects
        # if len(object_ids) > 1:
        #     for obj_id in object_ids[1:]:
        common_frames = set(common_frames)
        common_frames = common_frames.intersection(set(data[object_ids]['frames'].keys()))
        if len(common_frames) == 0:
            print("No common frames between objects")
            return
        common_frames = sorted(list(common_frames))
        # np array will be of the format T x (N(4 + N)) where T is the number of common frames and N is the number of objects
        # every (4 + N) columns will be for one object where the first 4 columns are for the object itself and the next N columns is a one hot encoding of 
        # which object it is
        # matrix = np.zeros((len(common_frames), len(object_ids) * (4 + len(object_ids))))
    for object_ids in triplets:
        matrix = np.zeros((len(common_frames), len(object_ids) * 4))
        umatrix = np.zeros((len(common_frames)-1, len(object_ids) * 2))
        prev_frame_xy = [(0, 0) for i in range(len(object_ids))]
        # populate the matrix
        for frame_idx, frame in enumerate(common_frames):
            for idx, obj_id in enumerate(object_ids):
                # get the object
                obj = data[obj_id]['frames'][frame]
                # get the index of the object in the matrix
                # obj_idx = idx * (4 + len(object_ids))
                obj_idx = idx * (len(object_ids)+1)
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
    with open('/home/rchandra/Research/MA_IRL/data/demo_trajectories_rld.pickle', 'wb') as file:
        pickle.dump({'x': demo_trajectories, 'u': u_trajectories}, file)    

    with open('data/demo_trajectories_udl.pickle', 'wb') as fileh:
        pickle.dump({'x': demo_trajectories, 'u': u_trajectories}, fileh)    


if __name__ == "__main__":
    # get input arguments
    # parser = argparse.ArgumentParser()
    # parser.add_argument("pickle_file", help="pickle data file to parse")
    # # parser.add_argument("object_ids", nargs="+", help="object ids to visualize")
    # args = parser.parse_args()
    # if args.pickle_file == None:
    #     print("Usage: python data_processing.py <pickle_file> <object_ids>")
    #     sys.exit(1)
    # prepare the interaction matrix
    # prepare_interaction_matrix(args.pickle_file)
    prepare_interaction_matrix("data/trajectories_clean_full_set")










