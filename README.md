# ChatRover: Control Mars Rover with Natural Language

ChatRover is a pioneering project that combines Language Logic Models (LLMs) with the control of rovers. This allows users to **steer rovers using common, everyday language commands.** The project runs on the ROS2 (Robot Operating System) Humble and utilizes the Gazebo environment for realistic simulations of rover behavior.

ChatRover is accompanied by a user-friendly web application. This provides an intuitive interface for users to deliver commands, bridging the gap between cutting-edge technology and user accessibility.

While ChatRover currently operates within a simulated environment for thorough testing and development, it is designed to translate natural language instructions into real-world commands for rovers when required.

Key features include:

- A user-friendly web application that provides an interactive interface for rover control
[![output-onlinegiftools.gif](https://i.postimg.cc/6qHmw0Sd/output-onlinegiftools.gif)](https://postimg.cc/0Kwf0p9Q)
  
- Full control over the rover's directional movement, including forward, backward, left, and right commands
[![output-onlinegiftools-2.gif](https://i.postimg.cc/bNrznFzf/output-onlinegiftools-2.gif)](https://postimg.cc/LndKKDYx)
  
- Support for commands in multiple languages

![rover-3](https://github.com/Gaurang-1402/ChatRover/assets/71042887/c26f704c-cc08-4194-9878-758491740a16)



## ROSGPT Architecture

![Screenshot from 2023-07-04 20-49-50](https://github.com/Gaurang-1402/ChatDrones/assets/71042887/f3534fd5-1ac8-4d55-8e67-fb5f6c0ddf8d)

1. The first component, "rosgpt.py", serves as the primary translator. As a REST server in a ROS2 node, it receives instructions in the form of POST requests, then processes these instructions into structured JSON commands using the ChatGPT API. Once the translation is complete, the commands are published on the /voice_cmd topic, ready for the next stage.

2. The next component is "rosgpt_client_node.py", a ROS2 client node that acts as a liaison between the user and rosgpt.py. It sends POST requests with the user's commands to the ROSGPT REST server and awaits the transformed JSON commands, displaying them upon receipt.

3. Another key component is "rosgpt_client.py", which fulfills a similar role to rosgpt_client_node.py. The main difference is that this script functions solely as a REST client for ROSGPT, without the ROS2 node involvement.

4. Once the commands are translated, they are received by "rosgptparser_rover.py". This script, dubbed the ROSGPTParser, executes the commands. It subscribes to the /voice_cmd topic, receives the JSON commands, parses them, and then carries out the necessary rover steering.

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
ros2 run rosgpt rosgptparser_rover
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
