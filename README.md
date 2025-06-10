<!-- PROJECT LOGO -->
<p align="center">
  <h2 align="center">TeraSim</h2>
  <p align="center">
  </p>
</p>

## Introduction

[TeraSim](https://michigan-traffic-lab.github.io/TeraSim-Landing-Page/) is a state-of-the-art traffic simulation environment meticulously designed for the precise evaluation of autonomous vehicle (AV) safety performance. Building upon the robust foundation of [SUMO](https://eclipse.dev/sumo/), TeraSim significantly enhances simulation fidelity with advanced traffic behavior modeling and high-precision maneuver execution capabilities.

A key feature of TeraSim is its co-simulation functionality, which enables seamless integration with open-source industry-standard tools such as [CARLA](https://carla.org/) and [Autoware](https://autoware.org/), facilitating comprehensive testing that combines TeraSim's advanced traffic modeling with high-fidelity sensor simulation and accurate vehicle dynamics.


## Installation

#### Requirements

For optimal performance and stability, we recommend installing the system on a dedicated machine rather than a virtual machine.

- __Hardware__: The Terasim simulation is not particularly resource-intensive. A system with an 8-core CPU and 8GB of RAM is typically sufficient, and a GPU is not necessary. However, for co-simulation with more demanding platforms such as CARLA, a more powerful system is advised.
- __System__: Ubuntu 22.04

#### Dependency Installation

- __Redis__: follow the instructions to install [Redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/). It is recommended to install [Redis commander](https://www.npmjs.com/package/redis-commander/v/0.6.3) a web-based Redis visualization tool.


- __Anaconda__: download and install [Anaconda](https://www.anaconda.com/download/success).


#### Clone the Repository
```bash
git clone https://github.com/michigan-traffic-lab/TeraSim.git
```

#### Setup a Virtual Environment
```
# Create a virtual environment
conda env create -f terasim-cosim.yaml

# Activate the virtual environment
conda activate terasim-cosim
```

## Usage

### Set up Redis
Set up a Redis server as an in-memory data structure store and leave it running in the background.

```
redis-server
```
### Run TeraSim

To control an autonomous vehicle (AV) with realistic vehicle dynamics, users can send control commands directly to CARLA within a co-simulation setup. These commands will be executed in CARLA, with the AV synchronized in TeraSim. For further details, refer to the section on Running CARLA Co-Simulation.

TeraSim can also operate independently. If CARLA co-simulation is not utilized, TeraSim will control the AV using the default Intelligent Driver Model (IDM).

The TeraSim Core is integrated into this repository and will be installed automatically. For more details, please refer to the [TeraSim Core repository](https://github.com/michigan-traffic-lab/TeraSim).

To run TeraSim, navigate to the terasim example directory.
```
cd examples/terasim_examples
```

TeraSim offers two modes for controlling background vehicles (BVs):

1. Run [Naturalistic and Adversarial Driving Environment (NADE)](https://www.nature.com/articles/s41467-021-21007-8)-based simulation
```
python3 terasim_nade_example.py
```
2. Run default SUMO simulation

```
python3 terasim_sumo_example.py
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
 
### TeraSim Closed-Loop Testing

For users interested in running TeraSim with Mcity real CAV fleet or Mcity ditigal twin in the CARLA simulator, check [Mcity-2.0-API-for-AV-motion-planning](https://github.com/michigan-traffic-lab/Mcity-2.0-API-for-AV-motion-planning).

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

## Developer

Zhijie Qiao zhijieq@umich.edu

Haowei Sun: haoweis@umich.edu

Haojie Zhu: zhuhj@umich.edu

Sean Shen - shengyin@umich.edu

## License

Distributed under the MIT License.

## Contact

- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab


