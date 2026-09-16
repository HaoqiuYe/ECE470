Procedure: Normal work flow Lab 2, 3, 4
In Terminal 1
1. delete all the previous build folders ,build, install, log
2. colcon build --symlink-install
3. source ur3_setup.sh
4. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.10.53 headless_mode:=true launch_rviz:=true kinematics_params_file:="Robot?_calibration.yaml"
  Change ? to your benches robot number. 

In Terminal 2
5. source ur3_setup.sh

6. ros2 run ece470labs lab2_exec
or 
6. ros2 run ece470labs lab3_exec 0 0 0 -90 0 0
or 
6. ros2 run ece470labs lab4_exec 0.1 0.1 0.15 90


Procedure: Normal work flow Lab 5
In Terminal 1
1. delete all the previous build folders ,build, install, log
2. colcon build --symlink-install
3. source ur3_setup.sh
4. ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.10.53 headless_mode:=true launch_rviz:=true kinematics_params_file:="Robot?_calibration.yaml"
  Change ? to your benches robot number. 
In Terminal 2
5. source ur3_setup.sh
6. ros2 run v4l2_camera v4l2_camera_node --ros-args --params-file ./codi_camera_full.yaml -p camera_info_url:="file:///opt/ros/jazzy/share/camera_calibration/Robot?_ost.yaml"
  Change ? to your benches robot number

In Terminal 3
7. source ur3_setup.sh
8. ros2 run ros2 run ece470labs lab5_exec


If you decide to use git for your backups at github.com
Steps to save your github credentials at the ECE470 Lab Linux machines:
1.  At FireFox log into github.com
2.  Create your repo and share it with your partner.  Make a empty repo
3.  Clone your empty repo to your home directory
4.  Copy the lab_files folder into your repo's folder
5.  run: "gh auth login" and select all the default options  (https:  and NOT ssh:)
6.  Now all your credentials are saved to Linux and you can git add,  git commit and git push, git pull without having to
reenter your passwords etc.  



DH Parameters for calculations of kinematics and dynamics
https://www.universal-robots.com/articles/ur/application-installation/dh-parameters-for-calculations-of-kinematics-and-dynamics/

e-series datasheet https://www.universal-robots.com/media/1807464/ur3e_e-series_datasheets_web.pdf

camera node: https://docs.ros.org/en/jazzy/p/v4l2_camera/
