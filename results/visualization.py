import sys, os
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import pandas as pd

def visualize_output(filename):
    df = pd.read_table(filename, sep='\t', lineterminator='\n')
    
    fig = plt.figure(figsize=(8,5), dpi=300)
    # plt.gca().set_aspect('equal')
    plt.title(os.path.basename(filename))
    plt.grid("on")
    plt.plot(df.px_ground_truth, df.py_ground_truth, lw=.5, label='Ground Truth')
    plt.plot(df.px_measured, df.py_measured, '.', color='red', ms=3., alpha=.75, label='Measurements')
    plt.plot(df.px_state, df.py_state, color='g', lw=.5, label='Estimates')
    plt.legend(loc=0)
    plt.xlabel("px")
    plt.ylabel("py")
    return fig

for arg in sys.argv[1:]:
    fig = visualize_output(arg)
    output_file = os.path.splitext(arg)[0]+".png"
    fig.savefig(output_file)
    print(output_file)