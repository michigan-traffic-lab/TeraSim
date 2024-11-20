# ROS2-Redis-Communication

## Introduction of the project

### About
This is a ROS2 package aimed to handle the communication between ROS2 server and Redis server.

### Features

### Code structure

## Installation

### Pre-requirements
```sh
sudo apt install ros-humble-rosbridge-suite
```

### Configuration
```sh
colcon build --symlink-install
```

## Usage

Please first source the Autoware.Universe workspace and this workspace:
```sh
    . /path/to/autoware/install/setup.bash
    . install/setup.bash
```
 Real vehicle side:
 ```sh
    export TERASIM_REDIS_HOST="44.222.171.64"
    export TERASIM_REDIS_PORT=6390
    export TERASIM_REDIS_PASSWORD="1G7R1SZDteJZmFa"
 ```
