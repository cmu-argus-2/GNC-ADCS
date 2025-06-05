# Argus CubeSat ADCS Algorithm Testing

This repository contains code and resources for testing Attitude Determination and Control System (ADCS) algorithms for the Argus CubeSat mission.

## Overview

The goal of this project is to develop, simulate, and validate ADCS algorithms tailored for the Argus CubeSat. The repository includes simulation scripts, test cases, and documentation to support robust algorithm development.

## Getting Started

Installing the simulation:
NOTE : The simulation only supports Ubuntu systems with a version >= 22.04
```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
git submodule init
git submodule update
sh install.sh
```
