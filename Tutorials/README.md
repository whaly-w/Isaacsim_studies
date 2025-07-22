# Tutorials for basic usage of standalone python

## Installation
- Download IsaacSim workstation follow [this instruction](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/download.html)
- Download IsaacSim using pip follow [this instruction](https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/pip_installation.html)
- Setup local assets follow [this instruction](https://docs.isaacsim.omniverse.nvidia.com/4.5.0/installation/install_faq.html#isaac-sim-setup-assets-content-pack)

## Utilization
Either activate virtual environment or use python.sh to run each stanalone .py file
```bash
# with env, assuming the env is called "env_isaaclab"
conda activate env_isaaclab
python Tutorials/Class01_Introduction.py 

# without env
./python.sh Tutorials/Class01_Introduction.py 
```
To open IsaacSim workstation, either 
```bash
# with env
isaacsim

# without env, open workstaion right away
./isaac-sim.sh 
# manage setup before launching workstation
./isaac-sim.selector.sh 
```

## Notes
- IsaacSim pip version require and **environment** like [miniconda](https://www.anaconda.com/docs/getting-started/miniconda/install)
- ROS2 bridge might not work in IsaacSim pip version 