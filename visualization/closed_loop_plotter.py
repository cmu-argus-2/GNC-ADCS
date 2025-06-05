import numpy as np
import os
import traceback
import inspect
import sys
# Get absolute path to 'fsw' directory
fsw_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
sys.path.append(fsw_path)
from visualization.closed_loop_plots import *
from argusim.visualization.parse_bin_file import parse_bin_file
import argparse

PERCENTAGE_TO_PLOT = 100

# Create a dictionary to hold method names and their corresponding functions
all_plotting_task_names = []
for name, func in inspect.getmembers(CLMontecarloPlots, predicate=inspect.isfunction):
    if not name.startswith("_"):
        all_plotting_task_names.append(name)
all_plotting_task_names = sorted(all_plotting_task_names)

    
if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        prog="plot.py",
        description="This program generates plots from data produced by a montecarlo job",
        epilog="=" * 80,
    )
    parser.add_argument("job_directory", metavar="job_directory")
    parser.add_argument("-t", "--trials", type=int, nargs="+")

    args = parser.parse_args()

    job_directory = os.path.realpath(args.job_directory)
    trials_directory = os.path.join(job_directory, "trials")
    plot_directory = os.path.join(job_directory, "plots")

    assert os.path.exists(job_directory), job_directory
    assert os.path.exists(trials_directory), trials_directory

    trials = sorted(
        [
            int(name.strip("trial"))
            for name in os.listdir(trials_directory)
            if os.path.isdir(os.path.join(trials_directory, name)) and name.startswith("trial")
        ]
    )

    if args.trials:
        for trial in args.trials:
            assert 1 <= trial
        trials = sorted(args.trials)
        plot_directory = os.path.join(job_directory + "_" + "_".join([str(x) for x in trials]), "plots")

    os.system(f"mkdir -p {plot_directory}")
    assert os.path.exists(plot_directory), plot_directory

    if not args.trials and os.path.exists(plot_directory) and os.listdir(plot_directory) != []:
        np.random.seed()
        passkey = np.random.randint(100, 1000)
        user_input = input(
            f'****WARNING****: Found exisitng plot folder for job "{args.job_directory}" at "{plot_directory}". Are you sure you want to risk overwriting existing plots? [Type {passkey} to continue] '
        )
        if user_input != str(passkey):
            print("exiting without plotting")
            exit(0)

    # plot_all(args.job_directory)
    mcp = CLMontecarloPlots(
        trials,
        trials_directory,
        plot_directory,
        PERCENTAGE_OF_DATA_TO_PLOT=PERCENTAGE_TO_PLOT,
        close_after_saving=True,
    )

    for plotting_task_name in all_plotting_task_names:
        try:
            plotting_task = getattr(mcp, plotting_task_name)
            plotting_task()
        except:
            traceback.print_exc()
        print()
    
