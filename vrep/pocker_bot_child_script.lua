

if (sim_call_type==sim.syscb_init) then 

    local moduleName=0
    local moduleVersion=0
    local index=0
    local pluginNotFound=true
    while moduleName do
        moduleName,moduleVersion=sim.getModuleName(index)
        if (moduleName=='RosInterface') then
            pluginNotFound=false
        end
        index=index+1
    end
    if (pluginNotFound) then
        -- Display an error message if the plugin was not found:
        sim.displayDialog('Error','RosInterface plugin not found. Run roscore before launching V-REP',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    exists, wheel_diameter= simROS.getParamDouble("wheel_diameter",0.087014) -- in m
    exists, wheelbase_m= simROS.getParamDouble("wheelbase_m",0.206)
    -- exists, map_frame=simROS.getParamString("map_frame","map")
    exists, odom_frame=simROS.getParamString("odom_frame","odom")
    exists, laser_frame=simROS.getParamString("laser_frame","laser_sensor")
    exists, base_frame=simROS.getParamString("base_frame","base_link")

    -- Robot handles
    robotHandle = sim.getObjectHandle('pocker_bot_base_link')
    motorLeft=sim.getObjectHandle('joint_left_wheel_link')
    motorRight=sim.getObjectHandle('joint_right_wheel_link')


    --Publishers
    odom_publisher = simROS.advertise('/odom','nav_msgs/Odometry')
    wheel_vel_pub = simROS.advertise('/wheel_vel','pocker_bot_vrep_description/wheel_vel')
    
    -- For reseting the bot based on topic /reset_pocker_bot
    reset_pose_flag = true

    --Robot wheel velocities
    left_velocity = 0.0
    right_velocity = 0.0

    --Robot position for odomhandle
    x_dist = 0.0
    y_dist = 0.0
    theta = 0.0
    last_time = 0.0

    -- TF frames
    tf_map2odom = false -- make it false if you dont want map to odom transform
    tf_odom2base = true -- make it false if you dont want odom to base transform

    -- print(odom_frame)
    map_frame = map_frame
    odom_frame = odom_frame
    base_frame = base_frame

-- Enable topic subscription:
    cmd_vel_sub = simROS.subscribe('/cmd_vel','geometry_msgs/Twist','cmd_callback')
    set_pose_sub = simROS.subscribe('/reset_pocker_bot','geometry_msgs/Pose2D','set_pose_callback')
end 


if (sim_call_type==sim.syscb_cleanup) then
    simROS.shutdownSubscriber(set_pose_sub)
    simROS.shutdownPublisher(odom_publisher) 
end 

if (sim_call_type==sim.syscb_actuation) then
    
    -- Send the robot's transform:
    publish_odom_and_tf()
    simROS.publish(wheel_vel_pub,{left_vel = left_velocity, right_vel = right_velocity})

end 

-- --function to set the target velocity of wheels
function set_wheel_target_velocity(lvel,rvel)
    sim.setJointTargetVelocity(motorLeft,lvel)
    sim.setJointTargetVelocity(motorRight,rvel)

    -- sim.addStatusbarMessage(string.format("left_vel:%f right_vel:%f",left_vel, right_vel))
end

-- function to convert cmd_vel to target velocities
function cmd_callback(msg) 
    lin_vel = msg.linear.x
    ang_vel = msg.angular.z
    left_velocity = lin_vel - (wheelbase_m/2)*ang_vel
    right_velocity = lin_vel + (wheelbase_m/2)*ang_vel
    
    reset_pose_flag = false -- means the bot is not reset to any other initial position
    set_wheel_target_velocity(left_velocity,right_velocity)
end


function yaw_to_quat(yaw)

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(0)
    sp = math.sin(0)
    cr = math.cos(0)
    sr = math.sin(0)

    quat ={}
    quat['w'] = (cy * cp * cr) + (sy * sp * sr)
    quat['x'] = (cy * cp * sr) - (sy * sp * cr)
    quat['y'] = (sy * cp * sr) + (cy * sp * cr)
    quat['z'] = (sy * cp * cr) - (cy * sp * sr)

    return quat

end

function publish_odom_and_tf()
    current_time = simROS.getTime();
    --dt = current_time - last_time
    dt = sim.getSimulationTimeStep()
    
    --sim.getObjectVelocity works at each simulation step and hence dt is taken as simulation time step and not the time difference
    if reset_pose_flag == false then
        lin_vel, ang_vel= sim.getObjectVelocity(robotHandle) --getting velocity of the body wrt world frame
    else
        lin_vel = {0,0,0}
        ang_vel = {0,0,0}
    end

    delta_x = (lin_vel[1]) * dt -- integrating velocities in time to find distance in x
    delta_y = (lin_vel[2]) * dt -- distance in y
    delta_th = ang_vel[3] * dt; -- theta angle
    --print (sim.getSimulationTimeStep())

    x_dist = x_dist + delta_x
    y_dist = y_dist + delta_y
    theta = theta + delta_th

    --print ("dt",dt)
    --print("delta_th : %s, theta: %s",delta_th,theta)

    orient_quat = yaw_to_quat(theta)
    --sim.addStatusbarMessage(string.format("quat: ",orient_quat))
    --print (theta)
    
    odom = 
    {
        header=
        {
            stamp= current_time,
            frame_id= odom_frame
        },
        child_frame_id= base_frame,
        pose=   
        {
            pose=  
            {
                position= {x=x_dist, y=y_dist, z=0},
                orientation= orient_quat
            }
        },
        twist=
        {
            twist=
            {
                linear= {x=lin_vel[1], y=lin_vel[2], z=lin_vel[3]},
                angular= {x=ang_vel[1], y=ang_vel[2], z=ang_vel[3]}
            }
        }
    }

    odom_tf=
    {
        header={
            stamp=current_time,
            frame_id=odom_frame
        },
        child_frame_id=base_frame,
        transform={
            translation={x=x_dist,y=y_dist,z=0},
            rotation=orient_quat
        }
    }

    simROS.publish(odom_publisher,odom)
    simROS.sendTransform(odom_tf)

   -- last_time = current_time
end

function set_pose_callback(msg)
    sim.resetDynamicObject(sim.handle_all)
    sim.setObjectPosition(robotHandle,-1,{msg.x,msg.y,(wheel_diameter/2)})
    sim.setObjectOrientation(robotHandle,-1,{0,0,msg.theta})
    sim.resetDynamicObject(sim.handle_all)
    theta = msg.theta
    reset_pose_flag = true
    -- return true
end