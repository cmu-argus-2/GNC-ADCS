

import os
import random
import time
from datetime import datetime

import numpy as np
from argusim.simulation_manager.sim import Simulator as cppSim

ARGUS_ROOT = os.getenv("ARGUS_ROOT", os.getcwd())
RESULTS_ROOT_FOLDER = os.path.join(ARGUS_ROOT, "results")
CONFIG_FILE = os.path.join(ARGUS_ROOT, "params.yaml")
trial = random.randint(0, 100)
RESULTS_FOLDER = os.path.join(RESULTS_ROOT_FOLDER, datetime.now().strftime("%Y-%m-%d_%H-%M-%S"))
os.mkdir(RESULTS_FOLDER)
try:
    cppsim = cppSim(trial, RESULTS_FOLDER, CONFIG_FILE, log=True)
except RuntimeError as e:
    print("stopping simulation")
