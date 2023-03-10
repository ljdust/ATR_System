# DYNAMIC CONNECTION HANDLING FOR SCALABLE ROBOTIC SYSTEMS USING ROS2
Authors: Lukas Dust, Emil Persson <br />
Supervisors: Per-Lage Götvall, Emmanuel Dean, Mikael Ekström, Saad Mubeen <br />
## Structure
The thesis can be found in Main branch in documentation folder <br />
The repository contains two branches: <br />
- Main branch: contains documentation and the latest implemented version
- Initial_Setup_Foxy branch: contains the first version of the simulation
See more information in the according sections or thesis work.<br />
## BRANCH MAIN
This branch contains the latest implemented version of the simulation running in ROS2 Galactic using Cyclone DDS

### Folder Structure
```
├── Documentation
|   ├── Images
│   |   └──DVA502_DVA503_Emil_Persson__Lukas_Dust.pdf
│   └── Images
|       └── ...
├── Code/src/ros2_ws
│   ├── atr_bot
|   |   ├── src
|   |   |   └── atr_bot.cpp
|   |   ├── CMakeList.txt
|   |   └── package.xml
│   ├── atr_bringup
|   |   ├── atr_bringup
|   |   |   ├── __init__.py
|   |   |   └── config_parser.py
|   |   ├── launch
|   |   |   ├── atr_launch.py
|   |   |   ├── atr_tracker_launch.py
|   |   |   ├── main_launch.py
|   |   |   └── visualisation_launch.py
|   |   ├── params
|   |   |   └── node_config_params.yaml
|   |   ├── CMakeList.txt
|   |   └── package.xml
│   ├── atr_interfaces
|   |   ├── include/atr_interfaces
|   |   |   └── ...
|   |   ├── msg
|   |   |   └── ...
|   |   ├── srv
|   |   |   └── ...
|   |   ├── CMakeList.txt
|   |   └── package.xml
│   ├── atr_tracker
|   |   ├── src
|   |   |   ├── atr_state_list_listener
|   |   |   └── atr_tracker.cpp
|   |   ├── CMakeList.txt
|   |   └── package.xml
│   └── atr_visualisation
|       ├── include/atr_visualisation
|       |   ├── ATRStateListSubscriber.h
|       |   └── AuxTools.h
|       ├── rviz
|       |   └── visualisation.rviz
|       ├── src
|       |   ├── Applications
|       |   |   └── atr_state_list_subscriber.cpp
|       |   ├── ATRStateListSubscriber.cpp
|       |   └── AuxTools.cpp
|       ├── urdf/meshes
|       |   └── ...
|       ├── CMakeList.txt
|       └── package.xml
└── README.md
```


### Prerequisites
- ROS2 Galactic
- Cyclone DDS (optional FastRTPS)

### System Overview
The system overview is given in the following figure:
![This is an image](/Documentation/Images/SimulationEnvironment.png)
#### Communication Architectures
Following two communication architectures are evaluated during this thesis:
![This is an image](/Documentation/Images/Architectures.png)
#### Node System
Following NODES are part of the system: 
```
├── atr_bot0
├── atr_bot...
├── atr_botn
├── atr_tracker
├── atr_state_list_listener
└── atr_visualisation
```
The functions of the nodes are:
 
#### Packages
Following ROS2 packages are defined:
```
├── atr_bot
├── atr_bringup
├── atr_interfaces
├── atr_tracker
└── atr_visualisation
```
where:
- __atr_bot__ contains the robot nodes
- __atr_bringup__ contains the launch files
- __atr_interfaces__ contains the service and message types
- __atr_tracker__ contains the tracker node and the measurement node
- __atr_visualisation__ contains the rviz configuration and the visualisation node
The division is made to allow a simple distribution of the nodes to different computers.

#### Timestamps
In the system overview there are presented four timestamps (T1 - T4) which are appended to the state message and state list.
The timestamps are taken:
- __T1__ at the creation of the status data in the robots individually
- __T2__ at the reception of the status data in the tracker
- __T3__ at the sending of the state list
- __T4__ at the reception of the state list


### System Configuration
To configure the system, a yaml configuration file can be used, which is located at:
```
└── src
    └── atr_bringup
        └── params
            └── node_config_params.yaml
```
Following parameters are from importance to the simulation:<br />
__log_file__ name of the file for creating measurements (the path is hardcoded in the state_list_listener node)<br />
__architecture__: (0,1) Communication architecture; 0 = many to one, 1 = one to one <br />
__num_of_atr_groups__: Number of launch groups<br />
__num_of_atr_in_groups__: Number of robots in each launch group<br />
__num_of_connected_atr__: Amount of total connected atr in the system<br />
__delay__: Delay in seconds between the launch of each robot group <br />
__start_id__: Id of the first robot in the network<br />
__state_publisher_period__: Period in ms for the publishing of the state inside the robots<br />
__base_position__: (x,y,z) Initial position of the robots (all will be spawned in a line from that position onwards)<br />
__debug__: (true,false) Enables the logging of the connection handling variables<br />
__list_publisher_period__: Period in ms for publishing the state list<br />
__watchdog_period__: Period in ms for watchdog timer<br />
__max_pos_age__: Maximum age of data in ms for absolute data consistency<br />

### System Build and Lauch
Set the configurations in the yaml config file. <br />
To build the system following command needs to be executed in the folder containing the src
```
colcon build --symlink-install --merge-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_CXX_STANDARD=17
```
To execute the ROS2 system, firstly the setup file from the build needs to be sourced, e.g. by calling:
```
source install/setup.bash
```
To launch the system, the main launch file can be called with: 
```
ros2 launch atr_bringup main_launch.py 
```

### Simulation and Measurement Services
There exists two services which can be used for simulation and for taking measurements.
#### State Service
The state service can be called from each launched robot individually and can set the robots to the following states:
1. Connection
2. Disconnection
3. Malfunction
4. Malfunction Repair

To call the service following function can be called in the terminal:
```
ros2 service call \wakeup/atr_bot_XX atr_interfaces/srv/WakeupSrv '{id: NN}'

```
where:
- __XX__ is the id of the robot to execute the service
- __NN__ is the id of the action to take <br />
```
Following parameters can be set for NN:
0 : The robot connects to the tracker (Connection)
1 : The robot disconnects from the tracker (Disconnection)
2 : The robot disconnects from the tracker without telling the tracker (Malfunction)
3 : The robot connects to the tracker without telling the tracker (Malfunction Repair)
```
#### Measurement Service
The measurement service is located in the state_list_listener node and can be called with the following command:
```
ros2 service call \Start_Measurement atr_interfaces/srv/StartMeasurementSrv '{window: XX}'
```
Where: <br />
-__XX__ is the amount of samples to take <br />
When a measurement is started, the timestamps for all connected robots will be printed to a csv file where the name and path is defined in the configuration file. One sample is one reception of the state list (Containing all four timestamps from creation of the state in the robots, to reception in the tracker, to sending of the list, to reception of the list)

## BRANCH INITIAL_SETUP_FOXY
Contains the reduced system simulation of the GPSS system without the dynamic connection handling using ROS2 Foxy and FastRTPS DDS. The system is __NOT__ providing the dynamic connection handling.
### Folder Structure
```
└── src
    ├── atr_bot
    ├── atr_bringup
    ├── atr_interfaces
    ├── atr_tracker
    └── atr_visualisation
```
### Prerequisites
- ROS FOXY
- FastRTPS DDS

### System Overview
The system overview is given in the following figure:
![This is an image](/Documentation/Images/Foxy.png)
### System Launch and Configuration
Launch and Configuration are the same as in Main branch



