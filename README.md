Welcome to the 2nd Laboratory of EFAC.
This repository is a supplimentary material for this lab: https://sites.google.com/view/utcn-efac/labs/lab2

Here you can find the scripts for the solution of this lab.
For the main part of the lab, you need to use the simulator as it is described in the documentation.

__________

# Simulator

Follow the tutorial at the EFAC website, until you can control the robot using your keyboard.
Clone this repo into catkin_ws/src/. ```git clone https://github.com/molnarszilard/wander_lab.git```
Run ```catkin_make```(in catkin_ws)
Run ```rosrun wander_lab wander```

__________

The addition of the repo is that you can try out the code in real life using a Pioneer 3-AT robot with a LiDAR LMS200 on top of it.

# Jetson NX on P3 with SickLMS (or PicoZense)

To start the demo run, get the necessary libraries (aria, ariacuda,rosaria, slicktoolbox), create a catkin workspace, clone this repo in its source, then run the catkin_make (+ source devel/setup.bash), then:

 - roslaunch wander_lab pioneer_start.launch

This command starts the connection between the robot, the lidar and the host machine. (you might want to wait about 30 seconds for the lidar to connect)
Then to check if you can control the robot, run

 - rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/RosAria/cmd_vel

and move around.

To use one of the scripts, make sure, that you are in control of the robot whatever happens to it depending on the code written, then run 

 - rosrun wander_lab p3_wander

To stop the robot press Ctrl+C.
You can implement other obstacle avoidance methods if you would like to.

_______
To run with Pico Zense Camera instead of the lidar. use the following commands:
 - roslaunch wander_lab pioneer_start_pico.launch

 - rosrun wander_lab p3_wander_pico

 (if you don't have the package for the pico zense, then clone it from: https://github.com/molnarszilard/pico_zense_camera)
 (if you are running it on jetson, inside the pico_zense_camera/src/pico_zense_camera/lib/ overwrite the files with the files from the zip there)
 (The program should work with the ADI Smart Cameras as well, although, not implemented explicitely, needs some adjusments here and there.)

___________

_______
#### Asus Xtion Camera + NX
To run with the Asus Xtion Camera you first need to clone the [openni2_camera](https://github.com/ros-drivers/openni2_camera) repo inside the `src` of the ros ws. After this is done, run:

- ```roslaunch wander_lab pioneer_start_asus.launch```

- ```rosrun teleop_twist_keyboard teleop_twist_keyboard.py /cmd_vel:=/RosAria/cmd_vel```

- ```rosrun wander_lab p3_wander_pico```

# Jetson Nano + Pico

-get the pico_zense_camera repo from https://github.com/molnarszilard/pico_zense_camera

inside its ws, source it, then run:

```roslaunch pico_zense_camera pz_camera.launch```

another terminal, same ws (without sourcing):

```roslaunch src/pico_zense_camera/launch/depth_laser_scan.launch```

another terminal, from home dir (or where jetson dir is):

```python catkin_ws/src/wander_lab/src/wander_jetson.py```

Notes: Camera is very unstable with Jetson Nano. Try different cables. THe camera should be powered externally (wall plug or another power bank). In my experience, It's better to run the Jetson Nano in 5W mode, but you can try with MAXW mode as well (if you have time for that sort of thing). The powerbank of the motor (sometimes of the camera) goes to sleep after a while (1-2 minutes), push the button on the powerbank to recover.


# Jetson Nano + Sphero

```sudo chmod 777 /dev/ttyTHS*```

```sudo chmod 777 /dev/ttyUSB*```

or

```ths``` if the alias is set

``` roslaunch wander_lab sphero.launch```
_______
# Setup and Additional information:

_______
Hardware setup for Jetson NX (make sure that you can reach the NX from through a wifi, before installing it to the robot configure it to connect automatically to your mobile hotspot):

connect the red/black power cable from the p3 into the power socket of the NX.
Start the P3 from the on/off switch behind the left-rear wheel.
connect the Lidar USB to the NX (solid grey cable).
connect the pioneer USB to the NX (seethrough grey cable). The order is important for the configurations (disrespecting it will cause additional configurations.)
check USB connection, there should be 0 and 1 if both the lidar and the robot is connected

 - ls -l /dev/ttyUSB*

add permissions

 - sudo chmod 777 /dev/ttyUSB*

_____________
(catkin_make && source devel/setup.bash)

connecting to robot:

 - rosrun rosaria RosAria
 (cable might be unstable) (port is 0 default, if USB was not 0, then: ‘rosrun rosaria RosAria _port:=/dev/ttyUSB1’)

(there is a slight chance, that you can only connect to the robot if you push it, so that the wheels are turning)

Lidar LMS200

requires direct serial connection between lidar and laptop 


do as it is written here:

https://github.com/jmscslgroup/sicktoolbox_wrapper

(maybe needed http://wiki.ros.org/sicktoolbox_wrapper/Tutorials/UsingTheSicklms )

 - rosrun sicktoolbox_wrapper sicklms _port:=/dev/ttyUSB0 _baud:=38400 _connect_delay:=30


(if it does not connect then add: ‘_connect_delay:=30’ to the end of the rosrun command)


___
install rosaria in a catkin:

http://wiki.ros.org/ROSARIA/Tutorials/How%20to%20use%20ROSARIA

installing Aria: sudo apt install aria2 libaria-dev

https://github.com/amor-ros-pkg/rosaria

short ros documentation, contains how to connect to … (better if downloaded)

https://github.com/MaoRodriguesJ/ROS-P3DX/blob/master/ros.pdf


(https://github.com/ros-drivers/sicktoolbox_wrapper)

## Sphero

RVR Sphero 

Follow  https://github.com/markusk/rvr 

and change 'sphero-sdk-raspberrypi-python' to https://github.com/DomsaVictor/jetson_sphero  

 

git clone https://github.com/ROBOTIS-GIT/hls_lfcd_lds_driver.git  

 

Change ‘/dev/ttyS0’ to  ‘/dev/ttyTHS1’ in file: /home/jetson/develop/rvr/ROS/catkin_workspace/src/rvr/lib/sphero_sdk/asyncio/client/dal/serial_async_dal.py  

Other information in the Trash detection notes in C24 Teams
