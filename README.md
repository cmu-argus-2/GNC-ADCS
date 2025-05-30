# Argus CubeSat ADCS Algorithm Testing

This repository contains code and resources for testing Attitude Determination and Control System (ADCS) algorithms for the Argus CubeSat mission.

## Overview

The goal of this project is to develop, simulate, and validate ADCS algorithms tailored for the Argus CubeSat. The repository includes simulation scripts, test cases, and documentation to support robust algorithm development.

## Features

- Modular ADCS algorithm implementations
- Simulation environment for algorithm validation
- Test cases for various mission scenarios
- Documentation and usage examples

## Getting Started

1. Clone the repository:
    ```bash
    git clone https://github.com/your-org/argus-adcs-testing.git
    ```
2. Install the simulation:
NOTE : The simulation only supports Ubuntu systems with a version >= 22.04
```bash
python3 -m venv .venv --system-site-packages
source .venv/bin/activate
git submodule init
git submodule update
sh install.sh
```
3. ?
4. Profit

## Contributing

Contributions are welcome! Please open issues or submit pull requests for improvements.

## License

This project is licensed under the MIT License.

## Contact

For questions or collaboration, please contact the Argus CubeSat team.