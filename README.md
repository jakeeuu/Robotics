# Robotics projects year 2024
Ballabio Giacomo, Bettiati Matteo, Calvi Mirko

## description
The first_project and second_project folders constains the cpp code to solve the problems explained in the relative power points.
Everything has been realized on ROS1 on a Docker Container.

## commands
To run the code follow those steps:
1. open a new docker container and run vnc : <code>./run_docker_vnc.sh</code>
3. run: for the first_project: <code> roslaunch first_project launch.launch </code>  
        for the second_project:  
            * to start the first task: <code> roslaunch second_project first_task.launch </code>
            * to start the second task: <code> roslaunch second_project second_task.launch </code>
4. open this [link](http://localhost:8080/vnc.html) and press connect
5. go back to the terminal and navigate to the folder containing the .bag file of the project you are executing (remember to unzip)
6. run: <code>rosbag play -l --clock <strong>name_of_the_bag.bag</strong></code>
7. enjoy