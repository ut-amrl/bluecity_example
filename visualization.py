import pickle
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
import argparse
import time,os, random

def filter_points(x, y):
    min_y = -10
    max_y = 15
    min_x = -20
    max_x = 20

    # slice y coordinates for vertical trajectories
    filtered_x = [xi for xi, yi in zip(x, y) if min_y <= yi <= max_y]
    filtered_y = [yi for yi in y if min_y <= yi <= max_y]

    # slice x coordinates for horizontal trajectories
    filtered_yy = [yi for xi, yi in zip(filtered_x, filtered_y) if min_x <= xi <= max_x]
    filtered_xx = [xi for xi in filtered_x if min_x <= xi <= max_x]
    return filtered_xx, filtered_yy

# Function to add noise to a value
def add_noise(value, noise_range):
    return value + random.uniform(-noise_range, noise_range)

# def visualize_trajectories():
def visualize_trajectories(pickle_file):
    # load the pickle file
    with open(pickle_file, 'rb') as f:
        trajectories = pickle.load(f)
        # trajectories is a dictionary mapping object id to a dictionary
        # of the form{'class': class_type, 'frames': {frame_number: {'x': x_coordinate, 'y': y_coordinate, 'speed': speed}}}
        # create a plot
        fig = plt.figure()
        # update the data for each line
        for obj_id in trajectories:
            x = [frame['x'] for frame in trajectories[obj_id]['frames'].values()]
            y = [frame['y'] for frame in trajectories[obj_id]['frames'].values()]
            objectType = int(trajectories[obj_id]['class'])
            if objectType < 10:  
                line, = plt.plot(x, y, label=obj_id, linestyle='solid')
            elif objectType == 10:  # pedestrian is 10, plot with dotted line
                line, = plt.plot(x, y, label=obj_id, linestyle='dotted')
            else:  # bicycle is 13, plot with dashed line
                line, plt.plot(x, y, label=obj_id, linestyle='dashed')
        legend = plt.legend(fontsize='x-large', title='Object ID', ncol=len(trajectories) // 10 + 1)
        # get name of pickle file without the path or extension
        pickle_file_name = pickle_file.split("/")[-1].split(".")[0]
        def export_legend(legend, filename=f"{pickle_file_name}_legend.png"):
            fig  = legend.figure
            fig.canvas.draw()
            bbox  = legend.get_window_extent().transformed(fig.dpi_scale_trans.inverted())
            fig.savefig(filename, dpi="figure", bbox_inches=bbox)
        export_legend(legend)
        # hide the legend
        legend.remove()
        # show the plot
        plt.show()
        # save the plot
        fig.savefig(f"{pickle_file_name}_graph.png")

if __name__ == '__main__':
        # get input arguments
    parser = argparse.ArgumentParser()
    parser.add_argument("pickle_file", help="pickle file to visualize")
    args = parser.parse_args()
    if args.pickle_file == None:
        print("Usage: python visualization.py <pickle_file>")
        sys.exit(1)
    # visualize the trajectories
    visualize_trajectories(args.pickle_file)
    

