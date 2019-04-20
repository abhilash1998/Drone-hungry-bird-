-- This script is used for realtime emulation of the environment in V-REP
function round(num, numDecimalPlaces)
  local mult = 10^(numDecimalPlaces or 0)
  return math.floor(num * mult + 0.5) / mult
end
function sysCall_init()
    -- Add required handles here
    handle = sim.getObjectHandle('Drone_Pos_Emulation')
    why={0,0,0}
    position_real = {}          -- create the matrix
    for i=1,3 do
      position_real[i] = {}     -- create a new row
    
    end
    
    
    -- Declaring required handles
    drone_handle = sim.getObjectHandle('Drone_Pos_Emulation')
    collection_handles= sim.getCollectionHandle('Obstacles')
    start = sim.getObjectHandle('initial_waypoint')
    hoop3_handle = sim.getObjectHandle('Position_hoop3')
    hoop1_handle = sim.getObjectHandle('Position_hoop1')
    hoop2_handle = sim.getObjectHandle('Position_hoop2')
    insect1 = sim.getObjectHandle('initial_waypoint1')
    insect2 = sim.getObjectHandle('initial_waypoint2')
    insect3 = sim.getObjectHandle('initial_waypoint3')

    -- Assigning obstacles handles
    no_of_obstacles = 1
    obstacles_handles = {hoop1_handle,hoop2_handle,hoop3_handle,sim.getObjectHandle('obstacle_1')}
    --[[obstacles_handles = {}
    for i=1,no_of_obstacles do
        table.insert(obstacles_handles,sim.getObjectHandle('obstacle_'..tostring(i)))
    end--]]

    -----------Add other required handles here----------------

    --goal_handle_2 = sim.getObjectHandle('goal_2')

    t=simOMPL.createTask('t')
    ss={simOMPL.createStateSpace('6d',simOMPL.StateSpaceType.pose3d,start,{-1.2,-1.2,0.225},{1.2,1.2,1.6},1)}
    simOMPL.setStateSpace(t,ss)
    simOMPL.setAlgorithm(t,simOMPL.Algorithm.RRTConnect)
    simOMPL.setCollisionPairs(t,{sim.getObjectHandle('eDrone_outer0'),collection_handles})



    no_of_path_points_required = 50

    compute_path_flag = true
    path_pub=simROS.advertise('/vrep/waypoints', 'geometry_msgs/PoseArray')    -- Publish the computed path under the topic /vrep/waypoints
    simROS.subscribe('/path_planning','std_msgs/Int64','sysCall_actuation')
    whycon_sub = simROS.subscribe('/whycon/poses', 'geometry_msgs/PoseArray', 'whycon_callback')
  
    
end

function getpose(handle,ref_handle)
    position = sim.getObjectPosition(handle,ref_handle)
    orientation = sim.getObjectQuaternion(handle,ref_handle)
    pose = {position[1],position[2],position[3],orientation[1],orientation[2],orientation[3],orientation[4]}
    return pose
end

function visualizePath( path )
    if not _lineContainer then
        _lineContainer=sim.addDrawingObject(sim.drawing_lines,1,0,-1,99999,{0.2,0.2,1})
    end
    sim.addDrawingObjectItem(_lineContainer,nil)
    if path then
        local pc=#path/7
        for i=1,pc-1,1 do
            lineDat={path[(i-1)*7+1],path[(i-1)*7+2],path[(i-1)*7+3],path[i*7+1],path[i*7+2],path[i*7+3]}
            sim.addDrawingObjectItem(_lineContainer,lineDat)
        end
    end
end

function packdata(path)

    local sender = {header = {}, poses = {}}
    
    sender['header']={seq=123, stamp=simROS.getTime(), frame_id="drone"}
    sender['poses'] = {}

    for i=1,((no_of_path_points_required-1)*7)+1,7 do
        a = {x = 0, y = 0, w = 0, z = 0}
        b = {x = 0, y = 0, z = 0}
        SFX=1.1/9.8
        SFY=-1.1/9.8
        SFZ=3/32
        why[1]=(path[i])/SFX
        why[1+1]=(path[i+1])/SFY
        why[1+2]=32-((path[i+2])/SFZ)
        
    for i=1,3 do
        if why[i] > 1 then
            why[i]=why[i]
        else
            why[i]=why[i]
        end
    end

        pose = {position = b, orientation = a, }
        pose.position.x = (why[1])
        pose.position.y = (why[1+1])
        pose.position.z = (why[1+2])
        sender.poses[math.floor(i/7) + 1] = pose
    end
    return sender
    
   
end

    -- Subscribing to the required topic



function sysCall_actuation()

end

function sysCall_sensing()
    -- put your sensing code here
end

function sysCall_cleanup()
    -- do some clean-up here
end



function compute_and_send_path(task)
    local r
    local path
    
    r,path=simOMPL.compute(t,10,-1,no_of_path_points_required)
    --print(r, #path)
    if(r == true) then
        visualizePath(path)
        message = packdata(path)  
        simROS.publish(path_pub,message)
        -- Provide slots for debug to cross check the message or path points you recieve after computing
    end
    return r
end
function sysCall_actuation(iter)


    compute_path_flag = false
    ---- Add your code to set start and goal state after getting present poses of the drone and the new goal when path_planning.py request you a path
    ---------------------------------------------------------------------------------------------------------------
    --iter=0
    iter=iter.data
    if iter==1 then
        
        start_handle=start
        goal_handle=insect1
        compute_path_flag = true
    end

    if iter==2 then
        start_handle= insect1
        goal_handle= insect2
        compute_path_flag = true
    end
    if iter==3 then
        start_handle=insect2
        goal_handle=insect1
        compute_path_flag = true
    end
    if iter==4 then
        start_handle=insect1
        goal_handle=insect3
        compute_path_flag = true
    end
    if iter==5 then
        start_handle=insect3
        goal_handle=insect1
        compute_path_flag = true
    end
    if iter==6 then
        start_handle=insect1
        goal_handle=start
        compute_path_flag = true
    end
       if compute_path_flag == true then
        -- Getting startpose
        start_pose = getpose(start_handle,-1)
        -- Getting the goalpose
        goal_pose = getpose(goal_handle,-1)
        -- Setting start state
        simOMPL.setStartState(t,start_pose)
        -- Setting goal state but the orientation is set same as that of startpose
        simOMPL.setGoalState(t,{goal_pose[1],goal_pose[2],goal_pose[3],start_pose[4],start_pose[5],start_pose[6],start_pose[7]})
        -- Computing path and publishing path points
        status = compute_and_send_path(t)
        if(status == true) then -- path computed
            compute_path_flag = false
        end
    end 


 end 
function whycon_callback(msg)
    -- Get the position of the real-world whycon marker and set the position of the drone in the simulator.
                wpos_x = round(msg.poses[1].position.x,3)
                wpos_y = round(msg.poses[1].position.y,3)
                wpos_z = round(msg.poses[1].position.z,3)
                --wpos_z = round(msg.poses[i].position.z,3)   
                SFX=1.1/9.8
                SFY=-1.1/9.8
                SFZ=3/32
                pos_x=wpos_x*SFX
                pos_y=wpos_y*SFY
                pos_z=(32-wpos_z)*SFZ
                position_real[1] = pos_x
                position_real[2] = pos_y
                position_real[3] = pos_z
                
            --print(i)
            ---print(position_real)
                sim.setObjectPosition(handle,-1,position_real)
end