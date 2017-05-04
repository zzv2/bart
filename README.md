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

- roscore
- roslaunch bart head_minimal.launch
- roslaunch bart server.launch

- http://localhost:8000/bart_web.html
- http://10.148.3.20:8000/bart_web.html