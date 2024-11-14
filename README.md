# TeraSim Cosim
TeraSim is a state-of-the-art traffic simulation environment meticulously designed for the precise evaluation of autonomous vehicle (AV) safety performance. Building upon the robust foundation of SUMO (Simulation of Urban Mobility), TeraSim significantly enhances simulation fidelity with advanced traffic behavior modeling and high-precision maneuver execution capabilities.

A key feature of TeraSim is its TeraSim-Cosim functionality, which enables seamless integration with industry-standard tools such as CARLA and Autoware, facilitating comprehensive testing that combines TeraSim's advanced traffic modeling with high-fidelity sensor simulation and accurate vehicle dynamics.


# Installation
## Requirements

- __System__: Ubuntu 22.04. To ensure reliable installation and operation, we recommend installing the system on a dedicated machine rather than on a virtual machine.&nbsp;

- __Hardware__: the Terasim simulation is not highly demanding in terms of computational power. A system with an 8-core CPU and 8GB of RAM is generally sufficient, and no GPU is required. However, if you plan to run additional software stacks alongside Terasim, consider upgrading the hardware to meet the additional requirements.

## Dependency Installation

- __Redis__: follow the instructions to install [redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/). It is also recommended that you install [redis commander](https://www.npmjs.com/package/redis-commander/v/0.6.3). a web-based redis visualization tool.


- __Anaconda__: download and install [Anaconda](https://www.anaconda.com/download/success).

## Package Installation
Create a new Conda environment named `terasim-cosim` and install all necessary packages within this environment.


#### Crete a virtual environment with Python 3.10
```bash
conda create -n terasim-cosim python=3.10
conda activate terasim-cosim
```

#### Clone the repository
```bash
git clone https://github.com/michigan-traffic-lab/TeraSim-Cosimulation.git
```

#### Install the packages
```
cd TeraSim-Cosimulation
sudo chmod +x install.sh
./install.sh
```

## Running the Simulation
Set up a redis server to store traffic data for co-simulation and leave it in the background.

```
redis-server
```

Navigate to the `example` folder and run `terasim_exapmle.py`. The SUMO window should open, and the simulation will start running.

```
cd examples
python3 terasim_example.py
```


## Troubleshooting

After the installation, if you encounter the following error and the SUMO gui does not show up:
```bash
libGL error: failed to load driver: swrast
X Error: code 2 major 152 minor 3: BadValue (integer parameter out of range for operation).
```
You can run the following command to fix the error (using Anaconda):
```bash
conda install -c conda-forge libstdcxx-ng
```

If you encounter an error: "_Could not create server TCP listening socket *:6379: bind: Address already in use_", that means the redis server is already running, and you can proceed without further action.

## Developer

Zhijie Qiao zhijieq@umich.edu
Haowei Sun: haoweis@umich.edu
Haojie Zhu: zhuhj@umich.edu

## License

Distributed under the MIT License.

## Contact

- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab


