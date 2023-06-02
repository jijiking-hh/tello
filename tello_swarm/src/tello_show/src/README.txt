1.运行gazebo
roslaunch hector_quadrotor_demo gazebo_tello_show_map.launch 
2.对gazebo_tello_zhidao解锁无人机
rosservice call /tello1/enable_motors "enable: true" 
rosservice call /tello2/enable_motors "enable: true" 
rosservice call /tello3/enable_motors "enable: true" 
rosservice call /tello4/enable_motors "enable: true" 
rosservice call /tello5/enable_motors "enable: true" 
rosservice call /tello6/enable_motors "enable: true"



