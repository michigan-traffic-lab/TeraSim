<!-- PROJECT LOGO -->
<p align="center">
  <h2 align="center">TeraSim-Cosimulation</h2>
  <p align="center">
  </p>
</p>

## Introduction

[TeraSim](https://github.com/michigan-traffic-lab/TeraSim) is a state-of-the-art traffic simulation environment meticulously designed for the precise evaluation of autonomous vehicle (AV) safety performance. Building upon the robust foundation of [SUMO](https://eclipse.dev/sumo/), TeraSim significantly enhances simulation fidelity with advanced traffic behavior modeling and high-precision maneuver execution capabilities.

A key feature of TeraSim is its co-simulation functionality, which enables seamless integration with open-source industry-standard tools such as [CARLA](https://carla.org/) and [Autoware](https://autoware.org/), facilitating comprehensive testing that combines TeraSim's advanced traffic modeling with high-fidelity sensor simulation and accurate vehicle dynamics.


## Installation

#### Requirements

- __Hardware__: the Terasim simulation is not highly demanding in terms of computational power. A system with an 8-core CPU and 8GB of RAM is generally sufficient, and no GPU is required. However, for co-simulation with more resource-intensive platforms like CARLA, a more powerful system is recommended.

- __System__: Ubuntu 22.04. To ensure reliable installation and operation, we recommend installing the system on a dedicated machine rather than on a virtual machine.


#### Dependency Installation

- __Redis__: follow the instructions to install [redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/). It is also recommended that you install [redis commander](https://www.npmjs.com/package/redis-commander/v/0.6.3). a web-based redis visualization tool.


- __Anaconda__: download and install [Anaconda](https://www.anaconda.com/download/success).

#### Package Installation
Create a new Conda environment named `terasim-cosim` and install all necessary packages within this environment.


#### Crete a Virtual Environment
```bash
conda create -n terasim-cosim python=3.10
conda activate terasim-cosim
```

#### Clone the Repository
```bash
git clone https://github.com/michigan-traffic-lab/TeraSim-Cosimulation.git
```

#### Install the Packages
```
# Change directory to TeraSim-Cosimulation
cd TeraSim-Cosimulation

# Grant execute permissions to the install.sh script for all users
sudo chmod +x install.sh

# Execute the install.sh script to start the installation process
./install.sh
```

## Running the Simulation
Set up a redis server to store traffic data for co-simulation and leave running it in the background.

```
redis-server
```

Navigate to the `example` folder and run `terasim_exapmle.py` with optional arguments. The SUMO window should open, and the simulation will start running. For detailed description of each argument, please refer to the python file. 

```
cd examples
python3 terasim_example.py --gui_flag --realtime_flag
```

## Configure the Simulation
Customizing the simulation requires some understanding of the SUMO environment. If you're new to SUMO, you can follow the [official tutorial](https://sumo.dlr.de/docs/index.html) for guidance.

#### Simulation Configuration Options

The simulation can be both **pre-configured** and **post-configured**:

- **Pre-configuration**: To adjust simulation settings, navigate to `examples/maps`. 
  - You can modify high-level configurations in `mcity_micro.sumocfg`.
  - To alter predefined vehicle routes and volumes, edit `mcity.route.xml`.

- **Runtime Configuration**: To make changes during the simulation, such as controlling the vehicle's movements or traffic lights based on real-time feedback:
  - Open `terasim_interface.py` and implement changes in the `user_step(traci)` function. This function is called and executed with each update of the simulator.
  - Examples for usage can be found in the `user_step_example(traci)` function, offering a useful starting point for implementing custom real-time actions.

 __IMPORTANT__: The testing environment is a fine-tuned Naturalist Driving Environment (NDE) designed for autonomous vehicle (AV) testing. For simulation stability, please configure only the AV's behavior through TraCI, while leaving the behavior of background vehicles (BVs) unchanged (BV information can still be accessed). If you choose not to directly control the AV, TeraSim's default IDM model will manage its behavior. If you wish to wish to customize all vehicles, please refer to our [Mcity-2.0-API-for-Joint-Control](https://github.com/michigan-traffic-lab/Mcity-2.0-API-for-Joint-Control/tree/main) repo which provides a generic TeraSim simulator setup.


## Running CARLA Co-Simulation
Download the [Mcity CARLA Simualtor](google.com). Extract the files and start running a CARLA server in the background.

```
./CarlaUE4.sh
```

Run the CARLA co-simulation script. You should now see all TeraSim vehicles synchronized into CARLA.
```
cd examples
python carla_cosim.py
```

## Troubleshooting

- **SUMO GUI not displaying**
    ```bash
    libGL error: failed to load driver: swrast
    X Error: code 2 major 152 minor 3: BadValue (integer parameter out of range for operation)
    ```
    You can resolve this issue by running the following command (for Anaconda users):
    ```bash
    conda install -c conda-forge libstdcxx-ng
    ```
- **Redis server creation error**
    ``` 
    "_Could not create server TCP listening socket *:6379: bind: Address already in use_",
    ```
    that means the redis server is already running, and you can proceed without further action.

## Developer

Zhijie Qiao zhijieq@umich.edu

Haowei Sun: haoweis@umich.edu

Haojie Zhu: zhuhj@umich.edu

Sean Shen - shengyin@umich.edu

## License

Distributed under the MIT License.

## Contact

- Sean Shen - shengyin@umich.edu - Michigan Traffic Lab
- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab


