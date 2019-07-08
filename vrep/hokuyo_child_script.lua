-- This is a ROS enabled Hokuyo_04LX_UG01 model (although it can be used as a generic 
-- ROS enabled laser scanner), based on the existing Hokuyo model. It performs instantaneous
-- scans and publishes ROS Laserscan msgs.



function sysCall_init() 
    laserHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01_ROS_laser")
    jointHandle=sim.getObjectHandle("Hokuyo_URG_04LX_UG01_ROS_joint")
    modelRef=sim.getObjectHandle("Hokuyo_URG_04LX_UG01_ROS_ref")
    modelHandle=sim.getObjectAssociatedWithScript(sim.handle_self)
    objName=sim.getObjectName(modelHandle)
    -- print (objName)
    -- communicationTube=sim.tubeOpen(0,objName..'_HOKUYO',1)
    exists, laser_frame=simROS.getParamString("laser_frame","laser_sensor")
    exists, angle_covered=simROS.getParamDouble("scan_angle_deg",180)
    laser_scan_pub = simROS.advertise('/scan','sensor_msgs/LaserScan') 

    scanRange=angle_covered*math.pi/180 --You can change the scan range. Angle_min=-scanRange/2, Angle_max=scanRange/2-stepSize
    stepSize=2*math.pi/1024
    pts=math.floor(scanRange/stepSize)
    print ("scanrange: ", scanRange, pts)
    dists={}
    points={}
    segments={}
    
    for i=1,pts*3,1 do
        table.insert(points,0)
    end
    --print (#points)
    for i=1,pts*7,1 do
        table.insert(segments,0)
    end
   --print (#segments)
    black={0,0,0}
    red={1,0,0}
    lines100=sim.addDrawingObject(sim.drawing_lines,1,0,-1,1000,black,black,black,red)
    points100=sim.addDrawingObject(sim.drawing_points,4,0,-1,1000,black,black,black,red)


    -- Check if the required plugin is there (libv_repExtRos.so or libv_repExtRos.dylib):
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
        sim.displayDialog('Hokuyo_URG_04LX_UG01_ROS error','ROS plugin was not found.&&nSimulation will not run properly',sim.dlgstyle_ok,false,nil,{0.8,0,0,0,0,0},{0.5,0,0,1,1,1})
    end

    -- topicName=simExtROS_enablePublisher('front_scan',1,simros_strmcmd_get_laser_scanner_data ,modelHandle,-1,'rangeDat'..objName) 
      

    -- parentTf=sim.getObjectHandle("parentTf#")  --get handle to parent object in tf tree. Change this to your needs
    -- tfname=simExtROS_enablePublisher('tf',1,simros_strmcmd_get_transform ,modelHandle,parentTf,'') --publish the tf
end

function sysCall_cleanup() 
    sim.removeDrawingObject(lines100)
    sim.removeDrawingObject(points100)
end 

function sysCall_sensing() 
    showLaserPoints=sim.getScriptSimulationParameter(sim.handle_self,'showLaserPoints')
    showLaserSegments=sim.getScriptSimulationParameter(sim.handle_self,'showLaserSegments')
    dists={}
    angle=-scanRange*0.5
    sim.setJointPosition(jointHandle,angle)
    jointPos=angle
    
    laserOrigin=sim.getObjectPosition(jointHandle,-1)
    modelInverseMatrix=simGetInvertedMatrix(sim.getObjectMatrix(modelRef,-1))
    
    for ind=0,pts-1,1 do
    
        r,dist,pt=sim.handleProximitySensor(laserHandle) -- pt is relative to the laser ray! (rotating!)
        m=sim.getObjectMatrix(laserHandle,-1)
        if r>0 then
            dists[ind]=dist
            -- We put the RELATIVE coordinate of that point into the table that we will return:
            ptAbsolute=sim.multiplyVector(m,pt)
            ptRelative=sim.multiplyVector(modelInverseMatrix,ptAbsolute)
            points[3*ind+1]=ptRelative[1]
            points[3*ind+2]=ptRelative[2]
            points[3*ind+3]=ptRelative[3]
            segments[7*ind+7]=1 -- indicates a valid point
        else
            dists[ind]=0
            -- If we didn't detect anything, we specify (0,0,0) for the coordinates:
            ptAbsolute=sim.multiplyVector(m,{0,0,6})
            points[3*ind+1]=0
            points[3*ind+2]=0
            points[3*ind+3]=0
            segments[7*ind+7]=0 -- indicates an invalid point
        end
        segments[7*ind+1]=laserOrigin[1]
        segments[7*ind+2]=laserOrigin[2]
        segments[7*ind+3]=laserOrigin[3]
        segments[7*ind+4]=ptAbsolute[1]
        segments[7*ind+5]=ptAbsolute[2]
        segments[7*ind+6]=ptAbsolute[3]
    
        ind=ind+1
        angle=angle+stepSize
        jointPos=jointPos+stepSize
        sim.setJointPosition(jointHandle,jointPos)
    end
    
    
    
    sim.addDrawingObjectItem(lines100,nil)
    sim.addDrawingObjectItem(points100,nil)
    
    if (showLaserPoints or showLaserSegments) then
        t={0,0,0,0,0,0}
        for i=0,pts-1,1 do
            t[1]=segments[7*i+4]
            t[2]=segments[7*i+5]
            t[3]=segments[7*i+6]
            t[4]=segments[7*i+1]
            t[5]=segments[7*i+2]
            t[6]=segments[7*i+3]
            if showLaserSegments then
                sim.addDrawingObjectItem(lines100,t)
            end
            if (showLaserPoints and segments[7*i+7]~=0)then
                sim.addDrawingObjectItem(points100,t)
            end
        end
    end
    
    
    -- Now send the data:
    -- if #points>0 then
        --table.insert(dists,-scanRange*0.5)  --append angle_min to [ranges]
        --table.insert(dists,scanRange*0.5-stepSize) --append angle_max to [ranges]
        --table.insert(dists,stepSize) --append stepsize to [ranges]
        -- The publisher defined further up uses the data stored in signal 'rangeDat'..objName
        -- sim.setStringSignal('rangeDat'..objName,sim.packFloatTable(dists))

    -- end
    range_scan = {}
    for i=1,#dists,1 do
        range_scan[i] = dists[i]
    end
    -- print (#range_scan)
    -- print (range_scan)
    -- print (#dists)
    t=simROS.getTime();
    -- scan_data = 
    scan_data = 
    {
        header=
        {
            seq=0,
            stamp= t,
            frame_id= laser_frame
        },
        angle_min= -scanRange/2,
        angle_max= scanRange/2,
        angle_increment= stepSize,
        time_increment= (50/pts)/1000,
        scan_time = t,
        range_min = 0.001, --to 
        range_max = 10, --same range is set in the properties of the proximity sensor
        ranges = range_scan,
        intensities = nil
    }
    --print (scan_data.ranges)
   -- print (#scan_data.ranges)
    --print ("scanrange: , stepSize: , pts: ,", scanRange, stepSize, pts)
    simROS.publish(laser_scan_pub,scan_data)
end 
