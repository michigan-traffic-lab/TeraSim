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

For optimal performance and stability, we recommend installing the system on a dedicated machine rather than a virtual machine.

- __Hardware__: The Terasim simulation is not particularly resource-intensive. A system with an 8-core CPU and 8GB of RAM is typically sufficient, and a GPU is not necessary. However, for co-simulation with more demanding platforms such as CARLA, a more powerful system is advised.
- __System__: Ubuntu 22.04

#### Dependency Installation

- __Redis__: follow the instructions to install [redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/). It is recommended to install [redis commander](https://www.npmjs.com/package/redis-commander/v/0.6.3) a web-based Redis visualization tool.


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

## Usage

#### Set up Redis
Set up a Redis server to store data for co-simulation and leave it running in the background.

```
redis-server
```

#### Run TeraSim

To focus on autonomous vehicle (AV) control using co-simulation, users should send AV control commands to CARLA, where they will be executed. The AV will also be synchronized in TeraSim. If CARLA co-simulation is not used alongside TeraSim, the AV will default to an IDM (Intelligent Driver Model) for control.

To start TeraSim, use the command below.
```
# Navigate to the example directory
cd examples
```

Two modes are provided:

1. Running [Naturalistic Driving Environment (NDE)](https://www.nature.com/articles/s41467-023-37677-5) -based simulation
```
python3 terasim_nde_example.py --gui_flag --realtime_flag
```
2. Running default SUMO simulation

```
python3 terasim_plain_example.py --gui_flag --realtime_flag
```

## Configure the Simulation
Customizing the simulation requires some understanding of the SUMO environment. If you're new to SUMO, you can follow the [official tutorial](https://sumo.dlr.de/docs/index.html) for guidance.

#### Simulation Configuration Options

The simulation can be both **pre-configured** and **post-configured**:

- **Pre-configuration**:
  - Modify the high-level sumo setting in `examples/maps/mcity_micro.sumocfg`.
  - Modify pre-defined vehicle routes and volumes in `examples/maps/mcity.route.xml`.

- **Runtime Configuration**: 
  - Read traffic data and control the vehicles in `terasim_user_functions.py`. Users should implement their changes in `user_step(traci)` function, which is called and executed with each update of the simulator. Some examples code has been given as a starting point.

 __IMPORTANT__: The NDE simulation is a fine-tuned environment designed for autonomous vehicle (AV) testing. To ensure simulation stability, avoid setting BV's actions using Traci, as this may lead to a simulation crash. BV information can still be accessed without issues.


## Running CARLA Co-Simulation
Download and extract the [Mcity CARLA Simualtor](google.com). Start a CARLA server in the background.

```
./CarlaUE4.sh
```

#### Running the AV Stack

Run the Autonomous Vehicle stack to spawn an AV capable of both manual and autonomous control.

```
cd examples
python3 carla_av_stack.py
```

#### Manual Control
1. Ensure the `pygame` window is active by moving the mouse cursor over it.
2. Use the **arrow keys** to drive the AV manually.
   - **Press `q`** to toggle reverse mode.

#### Autonomous Control
1. Modify the `send_av_control` function in the `terasim_user_functions.py` file to define the desired autonomous behavior.
2. While in manual mode, **press `p`** to switch to autonomous control.

#### Co-simulation
Run the CARLA co-simulation script to synchronize the AV and BVs.

```
cd examples
python3 carla_cosim.py
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

## Known Limitation
1. CARLA server crash: an intiric issue of CARLA. It is occurs, restart CARLA and everything else.
2. Background vehicle jittering in CARLA: this is due to the low update frequency of TeraSim (10Hz) and the asynchronous step of co-simulation.
3. Traffic light co-simulation between TeraSim and CARLA is currently not supported. You can still read traffic light status from TeraSim and implement your algorithms without visualization in CARLA.
4. Bicycle and Pedestrian co-simulation between TeraSim and CARLA is currently not supported. We expect to release this feature in the future.

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


