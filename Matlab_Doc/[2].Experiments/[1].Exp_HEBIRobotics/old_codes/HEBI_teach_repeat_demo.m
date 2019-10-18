%%
clear all;
close all;
clc

%% setting parmaeters
%ctrmode 1 : 모터에 position 지령(중력,coriolis등은 토크 지령으로)
%ctrmode 2 : 모터에 torque 지령
ctrmode = 2;
  
%holdspring 1 : drift를 kp 제어로 제거
%holdspring 2 : drit 무시
holdspring = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting   = 2;

%additionwaypoint 1 : addition waypoint with gravity compensation mode
%additionwaypoint 0 : use stored waypoint
additionwaypoint = 1;

%% HEBI setting
HebiLookup.initialize();

% kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper_KIMM.hrdf');
kin = setupArm('6dof_w_gripper');
gains = HebiUtils.loadGains('gains/6-DoF_arm_gains_KIMM[basic].xml');
trajGen = HebiTrajectoryGenerator();

familyName = 'Arm';%'6-DoF Arm';
moduleNames = {'Base','Shoulder','Elbow','Wrist1','Wrist2','Wrist3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames );
group.send('gains',gains);
cmd = CommandStruct();

% gripper setting
grippergroup = HebiLookup.newGroupFromNames('Gripper','Spool');
grippercmd = CommandStruct();

% Keyboard input
kb = HebiKeyboard();
           
%% gravity direction
% Assume gravity points down in the frame of the first module.  This will
% get used for gravity compensation when controlling the arm.  You can use
% the IMUs in the module to measure the actual orientation of the arm. See
% the advanced examples on how to do this.
if gravitysetting == 1
        gravityVec = [0 0 -1];
elseif gravitysetting ==2
    fbk = group.getNextFeedbackFull();
    baseRotMat = HebiUtils.quat2rotMat( [ 
        fbk.orientationW(1), ...
        fbk.orientationX(1), ...
        fbk.orientationY(1), ...
        fbk.orientationZ(1) ] );
    gravityVec = -baseRotMat(3,1:3);  
end

%% addition waypoint by gravComp
if additionwaypoint == 1
    velocityThreshold = 0.3;
    stiffness = 10 * ones(1,kin.getNumDoF());

    fbk = group.getNextFeedback();
    idlePos = fbk.position;
    stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

    disp('SPACE for waypoint w/o gripping, SHIFT for waypoint w/ girpping');

    waypoints = [];
    keys = read(kb);
    prevKeys = keys;

    isgripper = [];

    while keys.ESC == 0
        fbk = group.getNextFeedback();

        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );

        cmd.position = [];
        cmd.velocity = [];
        cmd.effort = gravCompEfforts + effortOffset; 

        if holdspring == 1
            % Find whether robot is actively moving
            isMoving = max(abs(fbk.velocity)) > velocityThreshold;
            if isMoving
                % Update idle position
                idlePos = fbk.position;
            else
                % Add efforts from virtual spring to maintain position
                driftError = idlePos - fbk.position;
                holdingEffort = driftError .* stiffness;
                cmd.effort = cmd.effort + holdingEffort;
            end  
        end
        group.send(cmd);

        % Add new waypoint on space bar press   
        keys = read(kb);
        if keys.SPACE == 1 && prevKeys.SPACE == 0 % diff state        
            waypoints(end+1,:) = fbk.position;
            isgripper = [isgripper 0];
            disp('SPACE pressed')
            disp('SPACE for waypoint w/o gripping, SHIFT for waypoint w/ girpping');
        elseif keys.SHIFT == 1 && prevKeys.SHIFT == 0 % diff state        
            waypoints(end+1,:) = fbk.position;
            isgripper = [isgripper 1];
            disp('SHIFT pressed')
            disp('SPACE for waypoint w/o gripping, SHIFT for waypoint w/ girpping');
        end
        prevKeys = keys;
    end
    disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints'])  
    
else %use previously stored waypoint
    waypoints = [ 0.0539    0.4924    2.2835    3.2184   -1.7094    5.2901;
                  0.4721    0.4923    2.1024    3.0223   -1.5737    5.2901;
                  0.0455    0.4939    2.2817    3.0490   -1.7910    5.2902];
    isgripper = [0 0 0];
end

%% Replay waypoints

group.startLog('dir','logs');

fbk = group.getNextFeedback(); 
timeToMove = 3;             % [sec]
time = [ 0 timeToMove ];    % [sec]

trajectory = trajGen.newJointMove([fbk.position; waypoints(1,:)], 'time', time );

% Initialize timer
t0 = fbk.time;
t = 0;

poscmdlog = [];
posfbklog = [];
%Move from the current position to first waypoint
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedback();
    t = fbk.time - t0;
    
    % Get new commands from the trajectory
    [pos,vel,acc] = trajectory.getState(t);
    poscmdlog = [poscmdlog; pos];
    posfbklog = [posfbklog; fbk.position];
    
    effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
    
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
    
    if ctrmode == 1
        cmd.position = pos;
        cmd.velocity = vel;
        cmd.effort = dynamicCompEfforts + gravCompEfforts + effortOffset;
    elseif ctrmode ==2 
        cmd.position = [];
        cmd.velocity = [];
        cmd.effort = -gains.positionKp.*(fbk.position - pos) +...% -gains.velocityKp.*(fbk.velocity - vel) + ...
                dynamicCompEfforts + gravCompEfforts + effortOffset;
    end
    
    group.send(cmd);
    
    if isgripper(1) == 1
        grippercmd.position = [];
        grippercmd.velocity = [];
        grippercmd.effort = -5;
    elseif isgripper(1) == 0
        grippercmd.position = [];
        grippercmd.velocity = [];
        grippercmd.effort = 1;
    end
    grippergroup.send(grippercmd);  
end

timeToMove = 1.5;             % [sec]
time = [ 0 timeToMove ];    % [sec]

% Split waypoints into individual movements
numMoves = size(waypoints,1);
for i = 2:numMoves

    % Pick start and end positions
    startPosition = waypoints(i-1,:);
    endPosition = waypoints(i,:);            
    trajectory = trajGen.newJointMove( [startPosition; endPosition], 'time', time );

    t0 = fbk.time;
    t = 0;            
    while t < trajectory.getDuration

        % Get feedback and update the timer
        fbk = group.getNextFeedback();
        t = fbk.time - t0;

        % Get new commands from the trajectory
        [pos,vel,acc] = trajectory.getState(t);
        poscmdlog = [poscmdlog; pos];
        posfbklog = [posfbklog; fbk.position];

        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);

        if ctrmode == 1
            cmd.position = pos;
            cmd.velocity = vel;
            cmd.effort = dynamicCompEfforts + gravCompEfforts + effortOffset;
        elseif ctrmode == 2
            cmd.position = [];
            cmd.velocity = [];
            cmd.effort = -gains.positionKp.*(fbk.position - pos) +...% -gains.velocityKp.*(fbk.velocity - vel) + ...
                    dynamicCompEfforts + gravCompEfforts + effortOffset;
        end

        group.send(cmd);
        
        if isgripper(i) == 1
            grippercmd.position = [];
            grippercmd.velocity = [];
            grippercmd.effort = -5;
        elseif isgripper(i) == 0
            grippercmd.position = [];
            grippercmd.velocity = [];
            grippercmd.effort = 1;
        end
        grippergroup.send(grippercmd);  
    end
  end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );