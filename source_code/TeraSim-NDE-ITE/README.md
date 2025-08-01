# TeraSim-NDE-ITE

## I. Introduction

### About
This repository provides a package for simulation naturalistic driving environment (NDE) and intelligent testing environment (ITE) in an arbitrary SUMO traffic network. It is developed by the [Michigan Traffic Lab](https://traffic.engin.umich.edu/) at the University of Michigan.


### Features
The code in this repository provides the following features:
- Simulation of NDE and ITE in an arbitrary SUMO traffic network
- Generation of NDE and ITE trajectories

### Code Structure
The code is organized into different modules or files. Here's a brief overview of their purpose:

- **envs**: This folder contains three different traffic environments: NDE, ITE and ITE with AV
- **vehicle**: This folder contains the vehicle model, including the negligence-based driver model and BV control model.

## II. Installation and Environment Configuration

### Pre-Requirements
TeraSim-NDE-ITE requires following packages to be installed: TeraSim (https://github.com/michigan-traffic-lab/TeraSim)

```bash
git clone https://github.com/michigan-traffic-lab/TeraSim
cd TeraSim
pip install -e .
cd ..
```
Also, please refer to the README of TeraSim for the installation of SUMO and other dependencies.

### Installation and Configuration Steps

```bash
git clone https://github.com/michigan-traffic-lab/TeraSim-NDE-ITE
cd TeraSim-NDE-ITE
pip install -e .
cd ..
```

## Usage

```bash
python ./example/safetest_mcity_main.py
```

## Contributing

Contributions to open-source projects are encouraged. Provide a template for this section, including instructions on how to fork the project and create feature branches.

## VIII. Developers

- Haowei Sun (haoweis@umich.edu)
- Haojie Zhu (zhuhj@umich.edu)

## IX. Contact

- Henry Liu (henryliu@umich.edu)

# Demo

A quick demo is provided to showcase how to run the code for the main purpose of the repository.

![Demo](./demo.gif)