# OdomErrorEstimator

​	This OdomErrorEstimator can estimate the errors of two standard ROS odometry messages,  output the error of two rotations,  position, linear and angular velocity.

## Configuration

​	You can modified the subscribe topic name to your required in mian.cpp file's line 159 and line 160.

## Run

​	git clone this repository into your ROS workspace‘s src folder, next cd your ROS workspace's root and run catkin_make

when compile is done, use command rosrun odomErrorEstimator odomErrorEstimator to run this node

## Output

​	The output format:

 - **Rotation Error**:

   The rotation error shows in eular angular format, which is convert from Quatation. If there are two ways to present the Quatation, use the one which not over 2 PI

 - **Position Error**:

   The positon error shows in sum of the squared errors of each position

 - **Linear Velocity Error&Angular velocity error**:

   The Linear Velocity Error&Angular velocity error show in sum of the squared errors of each linear and angular velocity

## Notice

​	This estimator require that the groundtruth and the esitmation odometrys' timestamp exactly the same

