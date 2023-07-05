# ChatRover: Control Mars Rover with Natural Language

ChatRover showcases the power of Language Logic Models (LLMs) by allowing users to control rovers using simple, natural language instructions. The system currently operates within a simulated environment for testing and development purposes.

Key features include:

- A user-friendly web application that provides an interactive interface for drone control
- Full control over the rover's directional movement, including forward, backward, left, and right commands
- Support for commands in multiple languages


## ROSGPT Architecture

![Screenshot from 2023-07-04 20-49-50](https://github.com/Gaurang-1402/ChatDrones/assets/71042887/f3534fd5-1ac8-4d55-8e67-fb5f6c0ddf8d)


## Getting started

Clone the repository

```
mkdir ros_ws
cd ros_ws
git clone <repo_url>
```

Install rosgpt libraries from the rosgpt folder

```
cd ~/src/rosgpt
pip3 install -r requirements.txt
```

Install ROS requirements

```
sudo apt-get install python-rosdep
sudo rosdep init
rosdep update
```

```
cd ~/ros_ws
rosdep install --from-paths src --ignore-src --rosdistro=<rosdistro> -y
```


Add your OpenAI API Key in your ```.bashrc``` as an environment variable 

```
echo 'export OPENAI_API_KEY=your_api_key' >> ~/.bashrc
```


## Running ROSGPT

First, navigate to the root directory of your workspace and build the project

```
cd ~/ros_ws
colcon build --symlink-install
```
Now run each of these commands on new terminals

```
source install/setup.sh
ros2 run rosgpt rosgpt
```

```
source install/setup.sh
ros2 run rosgpt rosgpt_client_node 
```

```
source install/setup.sh
ros2 run rosgpt rosgptparser_drone 
```

## Running the simulation

Moon simulation
```
source install/setup.sh
ros2 launch rover_gazebo moon.launch.py
```
![image](https://github.com/Gaurang-1402/ChatRover/assets/71042887/f066435f-3f23-467e-aca1-39ecc6e49b0c)


OR

Mars simulation

```
source install/setup.sh
$ ros2 launch rover_gazebo mars.launch.py
```

![image](https://github.com/Gaurang-1402/ChatRover/assets/71042887/2c6e6ecc-67c4-4375-b174-3a093dc25393)


OR

Forst simulation

```
source install/setup.sh
ros2 launch rover_gazebo forest.launch.py
```
![image](https://github.com/Gaurang-1402/ChatRover/assets/71042887/df88ae67-14c3-4a6c-8180-8b9f450e56b8)


Note: Please replace `<repository_url>` and `<your_api_key>` with the actual repository URL and your OpenAI API key, respectively.




## Credits
Simulation adapted from: https://github.com/mgonzs13/ros2_rover

```
@article{koubaa2023rosgpt,
  title={ROSGPT: Next-Generation Human-Robot Interaction with ChatGPT and ROS},
  author={Koubaa, Anis},
  journal={Preprints.org},
  year={2023},
  volume={2023},
  pages={2023040827},
  doi={10.20944/preprints202304.0827.v2}
}

```
