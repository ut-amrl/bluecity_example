import itertools
import pickle
# load a saved pickle file called trajectories.pickle, extract data into a dict called objects_dict
with open('trajectories.pickle', 'rb') as handle:
    objects_dict = pickle.load(handle)

# print(objects_dict.keys())

# find keys with values that are lists of length more than 1
# for key, value in objects_dict.items():
#     if len(value) > 1:
#         print(key)

# Get all possible combinations of 2 different IDs
id_combinations = itertools.combinations(objects_dict.keys(), 2)
# print the combinations
# for ids in id_combinations:
#     print(ids)

trajectories = {}
# Loop through the ID combinations
for ids in id_combinations:
    # for each ID, get all the data from objects_dict
    id1 = ids[0]
    id2 = ids[1]
    list1 = objects_dict[id1]
    list2 = objects_dict[id2]
    min_len = min(len(list1), len(list2))
    appended_data = [[list1[i], list2[i]] for i in range(min_len)]
    # In trajectories, create a new key with the ID combination. For the value, 
    
    trajectories[ids] = appended_data
# print(trajectories[('342773042', '380123343')])

with open('trajectories_MAIRL.pickle', 'wb') as handle:
    pickle.dump(trajectories, handle, protocol=pickle.HIGHEST_PROTOCOL)
