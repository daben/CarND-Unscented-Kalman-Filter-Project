import sys, os
import matplotlib
matplotlib.use('agg')
import matplotlib.pyplot as plt
import pandas as pd


def display_nis(filename):
    df = pd.read_table(filename, sep='\t', lineterminator='\n')
    df['time'] = (df.time_stamp - df.time_stamp.iloc[0]) * 1e-6

    df_lidar = df[df.sensor_type == 'lidar']
    df_radar = df[df.sensor_type == 'radar']

    fig, axes = plt.subplots(2, 1, figsize=(10, 10), dpi=300)
    ax = axes[0]
    ax.plot(df_radar.time, df_radar.NIS, lw=.5, label='NIS')
    ax.hlines(7.815, df_radar.time.min(), df_radar.time.max(), color='r', lw=2, label='95%')
    ax.legend(loc=0)
    ax.set_ylim(0, 20)
    ax.set_title("NIS RADAR")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("$\epsilon \sim \chi^2$")

    ax = axes[1]
    ax.plot(df_lidar.time, df_lidar.NIS, lw=.5, label='NIS')
    ax.hlines(5.991, df_lidar.time.min(), df_lidar.time.max(), color='r', lw=2, label='95%')
    ax.legend(loc=0)
    ax.set_title("NIS LIDAR")
    ax.set_xlabel("t [s]")
    ax.set_ylabel("$\epsilon \sim \chi^2$")
        
    return fig

for arg in sys.argv[1:]:
    fig = display_nis(arg)
    output_file = os.path.splitext(arg)[0] + "-nis.png"
    fig.savefig(output_file)
    print(output_file)