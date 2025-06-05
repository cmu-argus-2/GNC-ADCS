
from multiprocessing import Pool
import time
import os
from argusim.visualization.plots import MontecarloPlots
from visualization.adcs_plots import *
from argusim.visualization.parse_bin_file import parse_bin_file_wrapper
import yaml

# ANSI escape sequences for colored terminal output  (from ChatGPT)
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
WHITE = "\033[37m"
RESET = "\033[0m"  # Resets all attributes


class CLMontecarloPlots(MontecarloPlots):
    def __init__(
        self,
        trials,
        trials_directory,
        plot_directory,
        PERCENTAGE_OF_DATA_TO_PLOT,
        close_after_saving=True,
    ):
        super().__init__(
            trials,
            trials_directory,
            plot_directory,
            PERCENTAGE_OF_DATA_TO_PLOT,
            close_after_saving=close_after_saving,
        )

    def _plot_state_estimate_covariance(self, pyparams):
        if not pyparams["PlotFlags"]["attitude_determination_cov"]:
            return
        filepaths = self._get_files_across_trials("state_covariance.bin")

        START = time.time()
        args = [(filepath, self.PERCENTAGE_OF_DATA_TO_PLOT) for (_, filepath) in filepaths]
        with Pool() as pool:
            data_dicts = pool.map(parse_bin_file_wrapper, args)
        END = time.time()
        print(f"Elapsed time to read in data: {END-START:.2f} s")

        # ==========================================================================
        pyparams["NUM_TRIALS"]             = self.NUM_TRIALS

        plot_state_est_cov(pyparams, data_dicts, filepaths)
        
    def EKF_error_plots(self):
        with open(os.path.join(self.trials_dir, "../../../configs/params.yaml"), "r") as f:
            pyparams = yaml.safe_load(f)     
        # Maybe just append to pyparams instead?
        with open(os.path.join(self.trials_dir, "../../../configs/adcs_params.yaml"), "r") as f:
            pyparams.update(yaml.safe_load(f))
        
        if pyparams["PlotFlags"]["attitude_determination_error"]:
            filepaths = self._get_files_across_trials("fsw_state.bin")

            START = time.time()
            args = [(filepath, self.PERCENTAGE_OF_DATA_TO_PLOT) for (_, filepath) in filepaths]
            with Pool() as pool:
                data_dicts = pool.map(parse_bin_file_wrapper, args)
            END = time.time()
            print(f"Elapsed time to read in data: {END-START:.2f} s")
            # ==========================================================================  
            pyparams["trials"]                 = self.trials
            pyparams["trials_dir"]             = self.trials_dir
            pyparams["plot_dir"]               = self.plot_dir
            pyparams["close_after_saving"]     = self.close_after_saving
            pyparams["NUM_TRIALS"]             = self.NUM_TRIALS

            pyparams = EKF_err_plots(pyparams, data_dicts)

            # --------------------------------------------------------------------------
            self._plot_state_estimate_covariance(pyparams)  # show 3 sigma bounds

    def EKF_state_plots(self):
        with open(os.path.join(self.trials_dir, "../../../configs/params.yaml"), "r") as f:
            pyparams = yaml.safe_load(f)   
        with open(os.path.join(self.trials_dir, "../../../configs/adcs_params.yaml"), "r") as f:
            pyparams.update(yaml.safe_load(f))   
        
        if pyparams["PlotFlags"]["attitude_determination_states"]:
            # ======================= Estimated gyro bias =======================
            filepaths = self._get_files_across_trials("fsw_state.bin")

            START = time.time()
            args = [(filepath, self.PERCENTAGE_OF_DATA_TO_PLOT) for (_, filepath) in filepaths]
            with Pool() as pool:
                data_dicts = pool.map(parse_bin_file_wrapper, args)
            END = time.time()
            print(f"Elapsed time to read in data: {END-START:.2f} s")
              
            pyparams["trials"]                 = self.trials
            pyparams["trials_dir"]             = self.trials_dir
            pyparams["plot_dir"]               = self.plot_dir
            pyparams["close_after_saving"]     = self.close_after_saving
            pyparams["NUM_TRIALS"]             = self.NUM_TRIALS

            EKF_st_plots(pyparams, data_dicts, filepaths)
    
    # Possibly add plots for the remaining fsw states?
    # def fsw_state_plots(self):