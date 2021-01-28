import argparse, os, sys
import numpy as np

def parse_args():
    p = argparse.ArgumentParser(
        description="Draw SLAM maps in a single figure")
    p.add_argument('-dir', '--map_dir', type=str)
    p.add_argument('-fo', '--figure_output', type=str)
    return p.parse_args()

read_csv = lambda fn: np.genfromtxt(fn, delimiter=',')

def plot(data, plot_axis, plot_title, ax1, ax2):
    axis = plot_axis
    is_subplot = True

    axis.plot(data[:,ax1], data[:,ax2], label='ground truth',color='blue', linewidth=1)

    set_title = axis.title.set_text
    for item in axis.get_xticklabels() + axis.get_yticklabels():
        item.set_size(6)

    set_title(plot_title)
    axis.axis('equal')

def draw_slam_maps(map_dir, figure_output):
    import matplotlib.pyplot as plt
    cases = sorted(os.listdir(map_dir))
    n_per_row = int(np.ceil(np.sqrt(len(cases))))
    if n_per_row == 0:
        return
    n_rows = int(np.ceil(float(len(cases)) / n_per_row))
    figure, subplots = plt.subplots(n_rows, n_per_row, figsize=(15,10))
    subplots = np.ravel(subplots)

    for i in range(len(cases)):
        f = cases[i]
        map_fn = os.path.join(map_dir, f)
        case = f.split('.')[0]
        plot_axis = subplots[i]

        data = read_csv(map_fn)
        plot(data, plot_axis, case, 1, 2)

    for subplot in subplots[len(cases):]: subplot.axis('off')
    plt.tight_layout()
    plt.subplots_adjust(top=0.92)
    figure.savefig(figure_output)

if __name__ == '__main__':
    draw_slam_maps(**vars(parse_args()))
