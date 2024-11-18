<!-- PROJECT LOGO -->
<p align="center">
  <h2 align="center">TeraSim-Cosimulation</h2>
  <p align="center">
  </p>
</p>

## Introduction

[TeraSim](https://michigan-traffic-lab.github.io/TeraSim-Landing-Page/) is a state-of-the-art traffic simulation environment meticulously designed for the precise evaluation of autonomous vehicle (AV) safety performance. Building upon the robust foundation of [SUMO](https://eclipse.dev/sumo/), TeraSim significantly enhances simulation fidelity with advanced traffic behavior modeling and high-precision maneuver execution capabilities.

A key feature of TeraSim is its co-simulation functionality, which enables seamless integration with open-source industry-standard tools such as [CARLA](https://carla.org/) and [Autoware](https://autoware.org/), facilitating comprehensive testing that combines TeraSim's advanced traffic modeling with high-fidelity sensor simulation and accurate vehicle dynamics.

![Demo](Fig/cosim.mp4)

## Installation

#### Requirements

For optimal performance and stability, we recommend installing the system on a dedicated machine rather than a virtual machine.

- __Hardware__: The Terasim simulation is not particularly resource-intensive. A system with an 8-core CPU and 8GB of RAM is typically sufficient, and a GPU is not necessary. However, for co-simulation with more demanding platforms such as CARLA, a more powerful system is advised.
- __System__: Ubuntu 22.04

#### Dependency Installation

- __Redis__: follow the instructions to install [Redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/). It is recommended to install [Redis commander](https://www.npmjs.com/package/redis-commander/v/0.6.3) a web-based Redis visualization tool.


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

#### Run TeraSim

To control an autonomous vehicle (AV) with realistic vehicle dynamics, users can send control commands directly to CARLA within a co-simulation setup. These commands will be executed in CARLA, with the AV synchronized in TeraSim. For further details, refer to the section on Running CARLA Co-Simulation.

TeraSim can also operate independently. If CARLA co-simulation is not utilized, TeraSim will control the AV using the default Intelligent Driver Model (IDM).

The TeraSim Core is integrated into this repository and will be installed automatically. For more details, please refer to the [TeraSim Core repository](https://github.com/michigan-traffic-lab/TeraSim).

To start TeraSim, navigate to the example directory.
```
cd examples
```

TeraSim-Cosimulation offers two modes for controlling background vehicles (BVs):

1. Running [Naturalistic Driving Environment (NDE)](https://www.nature.com/articles/s41467-023-37677-5)-based simulation
```
python3 terasim_nde_example.py --gui_flag --realtime_flag
```
2. Running default SUMO simulation

```
python3 terasim_plain_example.py --gui_flag --realtime_flag
```

#### TeraSim Configuration Options

Customizing sumo-based TeraSim requires familiarity with SUMO environment. If you're new to SUMO, refer to [official tutorial](https://sumo.dlr.de/docs/index.html) for guidance.

The TeraSim (SUMO) supports both **pre-configured** and **runtime-configured**:

- **Pre-configuration**:
  - Adjust high-level SUMO setting in `examples/maps/mcity_micro.sumocfg`.
  - Modify pre-defined vehicle routes and traffic volumes in `examples/maps/mcity.route.xml`.

- **Runtime Configuration**: 
  - Read traffic data and control the vehicles in `terasim_user_functions.py`. Users should implement their changes in `user_step(traci)` function, which is called and executed with each simulation step. Sample code is provided to help you get started.

 __IMPORTANT__: The NDE simulation is a fine-tuned environment for autonomous vehicle (AV) testing. To ensure stability, avoid using TraCI to control BVs actions, as this may lead to simulation crashes. However, you can safely read background traffic data.
 
#### Running CARLA
If Carla co-simulation is used, users should send control commands directly to CARLA, where they will be executed, with the AV synchronized in TeraSim.

Download and extract the [Mcity CARLA Simualtor](https://drive.google.com/file/d/1MO1el1uwyudPVDsWDv531VS_7Pz98ovv/view?usp=sharing). Start a CARLA server in the background.

```
./CarlaUE4.sh
```
#### Set up Redis
Set up a Redis server to store data for co-simulation and leave it running in the background.

```
redis-server
```
#### Running the AV Stack

Launch the AV stack to spawn an AV that supports both manual and autonomous control:
```
cd examples
python3 carla_av_stack.py
```

#### Running AV sensors (optional)
For users needing to simulate perception data, we provide a template to generate LiDAR and varying camera images from CARLA and convert them to ROS2 format. To utilize this feature, users must install [ROS2](https://docs.ros.org/en/humble/index.html) and have a basic understanding of its framework. Detailed comments are included in the file to assist with setup and usage. The script can be executed using the following command:
```
cd examples
python3 carla_sensor_ros2.py
```

#### Manual Control
1. Ensure the `pygame` window is active by moving the mouse cursor over it.
2. Use the **arrow keys** to drive the AV manually.
   - **Press `q`** to toggle reverse mode.

#### Autonomous Control
1. Modify the `send_av_control` function in the `terasim_user_functions.py` file to define the desired autonomous behavior.
2. While in manual mode, **press `p`** to switch to autonomous control.

#### Ruuning Co-simulation
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
    That means the Redis server is already running, and you can proceed without further action.

## Known Limitation
1. CARLA Server Crashes: This is an inherent issue within CARLA. If it happens, simply restart CARLA along with any related processes.
2. Vehicle Jittering in CARLA: This occurs due to TeraSim's low update frequency (10Hz) combined with the asynchronous stepping in co-simulation.
3. Traffic Light Co-Simulation: Currently, traffic light synchronization between TeraSim and CARLA is not supported. However, you can still access traffic light status from TeraSim and utilize it in your algorithms without visual representation in CARLA.
4. Bicycle and Pedestrian Co-Simulation: Support for co-simulating bicycles and pedestrians between TeraSim and CARLA is not available at the moment. We plan to introduce this feature in the future.

## Developer

Zhijie Qiao zhijieq@umich.edu

Haowei Sun: haoweis@umich.edu

Haojie Zhu: zhuhj@umich.edu

Sean Shen - shengyin@umich.edu

## License

Distributed under the MIT License.

## Contact

- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab


