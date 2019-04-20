-- This script is used for realtime emulation of the environment in V-REP


function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end
function sysCall_init()
    why={0,0,0}
    --[[for i=1,3 do
        if why[i] > 1 then
            why[i]=why[i]
        else
            why[i]=why[i]
        end
    end]]--
    -- Add required handles here
    hoop1_handle = sim.getObjectHandle('Position_hoop1')
    hoop2_handle = sim.getObjectHandle('Position_hoop2')
    hoop3_handle = sim.getObjectHandle('Position_hoop3')
    obstacle1_handle = sim.getObjectHandle('obstacle_1')
    obstacle2_handle = sim.getObjectHandle('obstacle_2')
    start = sim.getObjectHandle('initial_waypoint')
    insect1 = sim.getObjectHandle('initial_waypoint1')
    insect2 = sim.getObjectHandle('initial_waypoint2')
    insect3 = sim.getObjectHandle('initial_waypoint3')
    hoop1=sim.getObjectHandle('Orientation_hoop3')
    hoop2=sim.getObjectHandle('Orientation_hoop4')
    hoop3=sim.getObjectHandle('Orientation_hoop1')

    position_real = {}          -- create the matrix
    for i=1,6 do
      position_real[i] = {}     -- create a new row
      for j=1,3 do
        position_real[i][j] = 0
      end
    end
    
    aruco_marker = {}          -- create the matrix
    for i=1,3 do
      aruco_marker[i] = {}     -- create a new row
      for j=1,3 do
        aruco_marker[i][j] = 0
      end
    end
    ite=0
    iite=0
    sinr_cosp=0
    cosr_cosp=0
    siny_cosp=0
    cosy_cosp=0
    sinp=0
    roll=0
    pitch=0
    yaw=0
    cy=0
    cp=0
    sr=0
    cr=0
    sy=0
    sp=0

    
    -- Subscribing to the required topics 
    aruco_sub = simROS.subscribe('/aruco_marker_publisher/markers', 'aruco_msgs/MarkerArray', 'aruco_callback')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
    key_input = simROS.subscribe('/input_key', 'std_msgs/Int16', 'key_callback')
    init_pt=simROS.advertise('/inpt', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
end



function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end


function packdata1(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    
    a = {x = 0, y = 0, w = 0, z = 0}
    b = {x = 0, y = 0, z = 0}
    SFX=1.1/9.8
    SFY=-1.1/9.8
    SFZ=3/32
    why[1]=(path[1])/SFX
    why[1+1]=(path[1+1])/SFY
    why[1+2]=32-((path[1+2])/SFZ)

    pose = {position = b, orientation = a, }
    pose.position.x = (why[1])
    pose.position.y = (why[1+1])
    pose.position.z = (why[1+2])
    sender.poses[math.floor(1/7) + 1] = pose

    return sender
    
   
end
function sysCall_actuation()
    
    startpt = getpose(start,-1)
    startpt = getpose(start,-1)
    startpt1=packdata1(startpt)
    simROS.publish(init_pt,startpt1)
    --print(startpt1)
end
function aruco_callback(msg)
   
    if key==1001 then        
        for i =1,3 do
                aruco_id = msg.markers[i].id
                orient_x = -round(msg.markers[i].pose.pose.orientation.x,3)
                orient_y = -round(msg.markers[i].pose.pose.orientation.y,3)
                orient_z = round(msg.markers[i].pose.pose.orientation.z,3)
                orient_w = round(msg.markers[i].pose.pose.orientation.w,3)
                --
    --[[ sinr_cosp = 2.0 * (orient_w * orient_x + orient_y * orient_z)
	 cosr_cosp = 1.0 - 2.0 * (orient_x * orient_x + orient_y * orient_y)
	roll = math.atan2(sinr_cosp, cosr_cosp)

	-- pitch (y-axis rotation)
	 sinp = 2.0 * (orient_w * orient_y - orient_z * orient_x)
	if (math.abs(sinp) >= 1)then
        if sinp>=0 then
        pitch=11/7
        else
        pitch=-11/7
        end
		-- use 90 degrees if out of range
	else
	
		pitch = math.asin(sinp)
    end
	 --yaw (z-axis rotation)
	 siny_cosp = 2.0 * (orient_w * orient_z + orient_x * orient_y)
	 cosy_cosp = 1.0 - 2.0 * (orient_y * orient_y + orient_z * orient_z)
	yaw = math.atan2(siny_cosp, cosy_cosp)
    
    pitch=pitch+180

     cy = math.cos(yaw * 0.5)
     sy = math.sin(yaw * 0.5)
     cp = math.cos(pitch * 0.5)
     sp = math.sin(pitch * 0.5)
     cr = math.cos(roll * 0.5)
     sr = math.sin(roll * 0.5)

    --Quaterniond q;
    orient_w = cy * cp * cr + sy * sp * sr
    orient_x = cy * cp * sr - sy * sp * cr
    oritn_y = sy * cp * sr + cy * sp * cr
    orient_z = sy * cp * cr - cy * sp * sr
    
        ]]--
        aruco_marker[i] = {orient_x,orient_y,orient_z,orient_w}
end 
        
        --print(aruco_marker)
               
        -- Get the orientation(quaternion) of the ArUco marker and set the orientation of the hoop using Orientation_hoop dummy
        -- Hint : Go through the regular API - sim.setObjectQuaternion
       
            sim.setObjectQuaternion(hoop1,-1,aruco_marker[1])
            --aruco_result[i+1]=sim.setObjectQuaternion(hoop2_handle,-1,aruco_marker[i])
            --sim.setObjectQuaternion(hoop3,-1,aruco_marker[2])
            sim.setObjectQuaternion(hoop2,-1,aruco_marker[2])
            sim.setObjectQuaternion(hoop3,-1,aruco_marker[3])
        key=100001
    end
end

function whycon_callback(msg)
    -- Get the position of the whycon marker and set the position of the food tree and non-food tree using Position_hoop dummy
    -- Hint : Go through the regular API - sim.setObjectPosition
   
      if key==1000 then 
                ite=ite+1
                for i=1,1 do
                
                    wpos_x = round(msg.poses[1].position.x,3)
                    wpos_y = round(msg.poses[1].position.y,3)
                    
                        
                    SFX=1.1/9.8
                    SFY=-1.1/9.8
                    SFZ=3/32
                    
                    pos_x=wpos_x*SFX
                    pos_y=wpos_y*SFY
                    pos_z=0.4959
                    offsety=0
                    offsetx=0
                    
                    position_real[1][1] = pos_x
                    position_real[1][2] = pos_y
                    position_real[1][3] = pos_z
                    if wpos_x<0 then
                        nwposx=wpos_x+offsetx
                    else 
                        nwposx=wpos_x-offsetx
                    end
                    if wpos_y<0 then
                        nwposy=wpos_y+offsety
                    else 
                        nwposy=wpos_y-offsety
                    end

                    pos_x=nwposx*SFX
                    pos_y=nwposy*SFY
                    wposz=32-((pos_z)/SFZ)-3.5                                      
                    pos_z=3-(wposz*SFZ)
                                        
                    position_real[2][1] = pos_x
                    position_real[2][2] = pos_y
                    position_real[2][3] = pos_z
                    --position_real[2][3] = 0.4959
                    --position_real[3][3] = 1.3345
                    --position_real[3][3] = 0.4959
                --print(i)
                    pos_x=wpos_x*SFX
                    pos_y=wpos_y*SFY
                    pos_z=0.6779
                    
                    position_real[3][1] = pos_x
                    position_real[3][2] = pos_y
                    position_real[3][3] = pos_z
                    if wpos_x<0 then
                        nwposx=wpos_x+offsetx
                    else 
                        nwposx=wpos_x-offsetx
                    end
                    if wpos_y<0 then
                        nwposy=wpos_y+offsety
                    else 
                        nwposy=wpos_y-offsety
                    end

                    pos_x=nwposx*SFX
                    pos_y=nwposy*SFY
                    
                    wposz=32-((pos_z)/SFZ)-3.5                                        
                    pos_z=3-(wposz*SFZ)
                                        
                    position_real[4][1] = pos_x
                    position_real[4][2] = pos_y
                    position_real[4][3] = pos_z
                    
                    pos_x=wpos_x*SFX
                    pos_y=wpos_y*SFY
                    pos_z=1.3345
                    
                    position_real[5][1] = pos_x
                    position_real[5][2] = pos_y
                    position_real[5][3] = pos_z

                    
                    wposz=32-((pos_z)/SFZ)-3.5                                        
                    pos_z=3-(wposz*SFZ)
                    if wpos_x<0 then
                        nwposx=wpos_x+offsetx
                    else 
                        nwposx=wpos_x-offsetx
                    end
                    if wpos_y<0 then
                        nwposy=wpos_y+offsety
                    else 
                        nwposy=wpos_y-offsety
                    end

                    pos_x=nwposx*SFX
                    pos_y=nwposy*SFY   
                    position_real[6][1] = pos_x
                    position_real[6][2] = pos_y
                    position_real[6][3] = pos_z
                    
                --print(position_real)
               if ite==1 then 
                    sim.setObjectPosition(hoop1_handle,-1,position_real[1])
                    sim.setObjectPosition(insect1,-1,position_real[2])
                    
                    key=10000
               end     
               if ite==2 then
                    sim.setObjectPosition(hoop2_handle,-1,position_real[3])
                    sim.setObjectPosition(insect2,-1,position_real[4])
                    --sim.setObjectPosition(hoop2_handle,-1,position_real[i+1])
                    --sim.setObjectPosition(hoop3_handle,-1,position_real[i+2])
                    key=10000
               end
               if ite==3 then
                    sim.setObjectPosition(hoop3_handle,-1,position_real[5])
                    sim.setObjectPosition(insect3,-1,position_real[6])
                    --sim.setObjectPosition(hoop2_handle,-1,position_real[i+1])
                    --sim.setObjectPosition(hoop3_handle,-1,position_real[i+2])
                    key=10000
               end
               
            end
            end
      if key==1002 then 
                iite=iite+1
                for i=1,1 do
                
                    wpos_x = round(msg.poses[1].position.x,3)
                    wpos_y = round(msg.poses[1].position.y,3)
                    
                        
                    SFX=1.1/9.8
                    SFY=-1.1/9.8
                    SFZ=3/32

                    pos_x=wpos_x*SFX
                    pos_y=wpos_y*SFY
                    pos_z=1.3250
                    
                    position_real[1][1] = pos_x
                    position_real[1][2] = pos_y
                    position_real[1][3] = pos_z
                end
                if iite==1 then
                    sim.setObjectPosition(obstacle1_handle,-1,position_real[1])
                    --sim.setObjectPosition(insect1,-1,position_real[2])
                end
                if iite==2 then
                    sim.setObjectPosition(obstacle2_handle,-1,position_real[1])
                    --sim.setObjectPosition(insect1,-1,position_real[2])
                end
                    
                    key=10000
               
        end 
      if key==1003 then 
                
                for i=1,1 do
                
                    wpos_x = round(msg.poses[1].position.x,3)
                    wpos_y = round(msg.poses[1].position.y,3)
                    
                        
                    SFX=1.1/9.8
                    SFY=-1.1/9.8
                    SFZ=3/32

                    pos_x=wpos_x*SFX
                    pos_y=wpos_y*SFY
                    pos_z=0.4959
                    
                    position_real[1][1] = pos_x
                    position_real[1][2] = pos_y
                    position_real[1][3] = pos_z
                end
                         
               
                    sim.setObjectPosition(start ,-1,position_real[1])
                   
                    --sim.setObjectPosition(hoop2_handle,-1,position_real[i+1])
                    --sim.setObjectPosition(hoop3_handle,-1,position_real[i+2])
                    key=10000
               
                    
        end 
                      
        
end

function getpose(handle,ref_handle)
    position1 = sim.getObjectPosition( handle,ref_handle)
    orientation1 = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position1[1],position1[2],position1[3],orientation1[1],orientation1[2],orientation1[3],orientation1[4]}
    return pose
end
function key_callback(msg)
    -- Read key input to set or unset position and orientation of food and non-food trees
    key=00
    key=msg.data
    print(key)
end