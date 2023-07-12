FROM ros:foxy
RUN apt-get update && apt-get install python3-serial screen -y

COPY ./ublox_ros2/ /ros2_ws/src/ublox_ros2/
COPY ./ublox_msgs/ /ros2_ws/src/ublox_msgs/

WORKDIR /ros2_ws/
RUN rosdep install -i --from-path src --rosdistro foxy -y
#RUN . /opt/ros/foxy/local_setup.sh
#RUN colcon build 
#RUN source install/setup.bash
