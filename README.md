<!-- PROJECT LOGO -->
<p align="center">
  <h2 align="center">TeraSim</h2>
  <p align="center">
  </p>
</p>

## Introduction

**TeraSim** is a state-of-the-art traffic simulation environment meticulously designed for the precise evaluation of autonomous vehicle (AV) safety performance. Building upon the robust foundation of [SUMO](https://eclipse.dev/sumo/), TeraSim significantly enhances simulation fidelity with advanced traffic behavior modeling and high-precision maneuver execution capabilities.

A key feature of TeraSim is its co-simulation functionality, which enables seamless integration with open-source industry-standard tools such as [CARLA](https://carla.org/) and [Autoware](https://autoware.org/), facilitating comprehensive testing that combines TeraSim's advanced traffic modeling with high-fidelity sensor simulation and accurate vehicle dynamics.

## Applications

🌟 If you're looking to systematically run TeraSim to test autonomous vehicle capabilities, we encourage you to explore our [Behavioral Safety Assessment](https://github.com/michigan-traffic-lab/Behavioral-Safety-Assessment/tree/main).


🌟 If you'd like to test your AV algorithm using the TeraSim environment on the Mcity testing track, please take a look at our [Mcity 2.0 Project](https://mcity.umich.edu/what-we-do/mcity-test-facility/remote-access/).


🌟 If you'd like to conduct closed-loop, full-stack AV testing simulation in CARLA using TeraSim, check out our [Mcity-2.0-API-for-AV-motion-planning](https://github.com/michigan-traffic-lab/Mcity-2.0-API-for-AV-motion-planning).


## Installation

#### Requirements

- __Hardware__: The Terasim simulation is not particularly resource-intensive. A system with an 8-core CPU and 8GB of RAM is typically sufficient, and a GPU is not necessary. However, for co-simulation with more demanding platforms such as CARLA, a more powerful system is advised.
- __System__: Ubuntu 22.04 (
For optimal performance and stability, we recommend installing the system on a **dedicated machine** rather than a virtual machine).

#### Dependencies

- __Redis__: follow the instructions to install [Redis](https://redis.io/docs/latest/operate/oss_and_stack/install/install-redis/install-redis-on-linux/).


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

To run TeraSim, navigate to the example directory.
```
cd examples
```

TeraSim offers multimodal examples:

1. [Naturalistic and Adversarial Driving Environment (NADE)](https://www.nature.com/articles/s41467-021-21007-8) simulation: `python3 safetest_nade_example.py`

2. Default SUMO simulation: `python3 default_sumo_example.py`

3. Pedestrian simulation: `python3 pedestrian_example.py`

4. Cyclist simulation: `python3 cyclist_example.py`

5. Construction zone simulation: `python3 construction_example.py`

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

Sean Shen: shengyin@umich.edu

## License

Distributed under the MIT License.

## Contact

- Henry Liu - henryliu@umich.edu - Michigan Traffic Lab


