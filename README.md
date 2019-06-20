# Pocker_bot_description
Run launch file `urdf_rviz.launch` to visualize the urdf model in rviz.
![alt text](tf/tf_frame.png)

You can find that the axis of the wheels are different compared to usual urdf. This is because, in vrep the joints are in similiar direction. Hence the axis of rotation in this case will be z axis rather than y axis in usual cases.

On launching `urdf_rviz.launch`, the robot_description is added to the parameter server and robot_state_publisher publishes the tf. In V-REP side you have to write a joint_state_publisher to publish its wheel velocity.

