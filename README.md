# bart
HRI 4410 Final Project


## Setup
- sudo apt-get install ros-indigo-rosbridge-*
- sudo apt-get install python-numpy python-scipy python-matplotlib

- cd ~/ros_ws/src
- git clone https://github.com/zzv2/bart.git


## Usage
- plug in robot to wall and usb
- make sure laptop is connected to RedRover
- open the terminal (use Ctrl+Shift+T to open a new tab)
- run the next three commands in their own terminal:
	- roscore
	- roslaunch bart head_minimal.launch
	- roslaunch bart server.launch

- verify that the ip address of the server is 10.148.3.20
- navigate to this url in a browser:
- http://10.148.3.20:8000/bart_web.html