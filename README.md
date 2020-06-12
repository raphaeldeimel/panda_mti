# MTI Panda Controllers

This package includes a set of lean controllers for Franka-Emika's Panda robot arms

Features:
- Joint space PD controller with full gains matrices
- integration of gripper into robot state (7+1 DoFs)#
- integration of a wrist F/T sensor into the control loop
- control and monitoring topics interface to ROS
- automatic enable/disable using the activation button
- Watchdog: goes into a gravity-compensation mode if no goals are received for some time
- soft joint limits: add repulsive forces to avoid violating joint limits during compliant motion 
- compensate torque bias, stiction and friction, load values from yaml based on MAC address (per-robot config)
- partially compensate inertias / decouple inertias of joints
- no compile/runtime dependency on franka_ros, only needs libfranka installed
- Include dynamics parameters from IROS'19 system identification paper


## Special installation notes for Ubuntu 20.04

In Ubuntu 20.04, the package of PyKDL 1.4.0-7 is broken (exception on unicode string conversion). One can work around though by installing debian's updated package:

wget http://deb.debian.org/debian/pool/main/o/orocos-kdl/liborocos-kdl1.4_1.4.0-9_amd64.deb <http://deb.debian.org/debian/pool/main/o/orocos-kdl/python3-pykdl_1.4.0-9_amd64.deb>
sudo dpkg --install liborocos-kdl1.4_1.4.0-9_amd64.deb

wget http://deb.debian.org/debian/pool/main/o/orocos-kdl/liborocos-kdl-dev1.4_1.4.0-9_amd64.deb <http://deb.debian.org/debian/pool/main/o/orocos-kdl/python3-pykdl_1.4.0-9_amd64.deb>
sudo dpkg --install liborocos-kdl-dev1.4_1.4.0-9_amd64.deb

wget http://deb.debian.org/debian/pool/main/o/orocos-kdl/python3-pykdl_1.4.0-9_amd64.deb <http://deb.debian.org/debian/pool/main/o/orocos-kdl/python3-pykdl_1.4.0-9_amd64.deb>
sudo dpkg --install python3-pykdl_1.4.0-9_amd64.deb 


## Package controller_panda_mti

The controller package contains several controllers that combine different hardware setups

pdcontroller: Controls only the robot arm 
gripper:      Also manage gripper 
netft:        Also read in netft data 

## Package msgs_panda_mti

Contains the ROS message API to the PD controller node.

We use monolithic, custom robot status and pd control goal messages, so all data of a time step are sent together for simpler sychronization.

To convert robot status messages to standard ROS joint messages (e.g. to convert those to link frames), use controller2jointstatemsgspublisher node from controller_panda_mti 

## Package panda_dynamics_model

Contains a URDF model annotated with dynamics parameters, and a small python class to compute Jacobians, Inertia matrices etc. 

## Package franka_description_mti

Package containing the meshes and URDFs for the Panda robot

Note: This package is derived from Franka-Emika's fanka_ros, because of this it uses a different licence (Apache) than all the other packages in this repository (BSD)

Differences to franka_ros:
    - Support for multiple robot arms / multiple poses of the robot arm
    - Added dynamics parameters to URDF
    - copy avoids dependency on the complete franka_ros stack


## Package panda_friction

Package to identify friction parameters of an arm automatically


### Calibrate Friction Parameters

1. get the mac address of the robot you want to calibrate. 

```bash
arp -a panda  | awk '{print $4}' | sed s/://g | tr -d '\n'
```

2. Go to /controller/config/ and open corresponding config file

3. Run controller 

```bash
roslaunch controller_panda_mti test_pdcontroller.launch
```

4. Adjust torque Bias

    a) Check whether there is constant movement in any joint.
    b) Adjust corresponding joint value higher or lower opposing motion.
    c) Reapeat until robot does not move at all. 

5. Adjust torque_stiction

    a) Move every joint by hand and check whether there is to much frictional force opposing the motion.
    b) Increase or decrease corresponding joint value
    c) Repeat until every joint is easy to move by hand. 
    
    
## Licences

Please note that the subdirectory franka_description_mti is dervied from franka_ros
https://github.com/frankaemika/franka_ros/ . It is licenced under Apache 2.0, wheras the rest of the repository is licenced under a BSD license (see LICENCE for details)


