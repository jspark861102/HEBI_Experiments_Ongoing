%��� ���� ���� ����
%�߷� ���� ��������, Ű���� Ȱ���� waypoint ������ �߰��� ������
%%
clear all;
close all;
clc

%% setting parmaeters
%ctrmode 1 : ���Ϳ� position ����(�߷�,coriolis���� ��ũ ��������)
%ctrmode 2 : ���Ϳ� torque ����
ctrmode = 2;
  
%holdspring 1 : drift�� kp ����� ����
%holdspring 2 : drit ����
holdspring = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting   = 2;

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

%% HEBI setting
% HEBI_Startup;

HebiLookup.initialize();
[kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;

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
velocityThreshold = 0.3;
stiffness = 10 * ones(1,kin.getNumDoF());

fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

disp('Add waypoint with SPACE.  Exit teaching mode with ESC');

waypoints = [];
keys = read(kb);
prevKeys = keys;

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
        disp('Enter next waypoint with SPACE.  Exit teaching mode with ESC');
    end
    prevKeys = keys;
end

%% Replay waypoints
disp(['Replaying ' num2str(size(waypoints,1)) ' waypoints'])

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
end

timeToMove = 1.5;             % [sec]
time = [ 0 timeToMove ];    % [sec]

%Move along waypoints
% while true
for iter = 1 : 1
    if stopBetweenWaypoints
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

               true effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

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
            end 
        end
        
        keys = read(kb);    
        if keys.ESC == 1
            break;
        end

    else
        trajectory = trajGen.newJointMove(waypoints, 'time', time );
            
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
        end
        
        keys = read(kb);
        if keys.ESC == 1
            break;
        end          
    end
    
    % Go home
    startPosition = waypoints(end,:);
    endPosition = waypoints(1,:);
    trajectory = trajGen.newJointMove( [startPosition; endPosition], 'time', time );

    t0 = fbk.time;
    t = 0;    
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
    end

end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();
% HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
% HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
% HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );

waypoints = waypoints
xyztargets = [];
for i = 1 : size(waypoints,1)
xyztargets = [xyztargets kin.getFK('endeffector',waypoints(i,:))];
end
xyztergets = xyztargets


%% display
% waypoints_joint = waypoints
% waypoints_EE = kin.getFK('endeffector', waypoints)