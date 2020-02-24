% ���κ� ���� �ʱ� �ܰ趧 ���Ϸν�, ������ ��ġ (�簢��)���� ���ʴ�� ��ġ ���� �ϴ� ����
% ��� �κ� ������ ��ġ,�ӵ�,��ũ ���۷����� Ȱ���ѵ� ����, ���� ��ũ ���۷����� Ȱ���Ͽ� ������
% �ڵ��� ����, �׸��� ���� ���� �����Ǿ� ����
% ���� (joint space) ������ �浹������ �����Ǿ� ����

%%
clear *;
close all;
clc

%% setting parmaeters
%ctrmode 1 : ���Ϳ� position ����(�߷�,coriolis���� ��ũ ��������)
%ctrmode 2 : ���Ϳ� torque ����
ctrmode = 2;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

%is there IO?
isIO = 0;

%% Target Waypoints
% xyzTargets = [ 0.40  0.60  0.60  0.40;    % x [m]
%                0.15  0.15 -0.15 -0.15;    % y [m]
%                -0.10  -0.10  -0.10  -0.10 ];  % z [m]
           
xyzTargets = [ 0.60   0.60   0.40   0.40;    % x [m]
              -0.20  -0.20  -0.20  -0.20;    % y [m]
              -0.10   0.10   0.10  -0.10 ];  % z [m]

%% HEBI setting
HebiLookup.initialize();
[kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;
group.startLog('dir','logs');

if isIO == 1;
    % Virtual I/O setting
    IOgroup = HebiLookup.newGroupFromNames('HEBI','Virtual IO');
end
    
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

%% trajectory & control
% Rotation matrix that makes the end-effector point straight forward
rotMatTarget = R_y(pi/2);   % [3x3 SO3 Matrix]
              
initPosition = [ 0 pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]

% Do IK to get joint position waypoints for each XYZ target, as well as the 
% desired orientation of the end effector.  Copy the first waypoint at the 
% end, so that it closes the loop
for i=1:length(xyzTargets(1,:))
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                 'SO3', rotMatTarget, ...
                                 'initial', initPosition ); 
end
posTargets(end+1,:) = posTargets(1,:);
posTargets(end+1,:) = posTargets(1,:);

% Get the initial feedback joint positions, and go from there to the first
% waypoint, using the trajectory API.  
fbk = group.getNextFeedback();
waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
timeToMove = 5;             % [sec]
time = [ 0 timeToMove ];    % [sec]

% Calculate initial trajectory to starting posiiton
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Initialize timer
t0 = fbk.time;
t = 0;

poscmdlog = [];
posfbklog = [];
group.send(CommandStruct());
% Execute the motion to go to the first target
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedback();
    t = fbk.time - t0;
    
    % Get new commands from the trajectory
    [pos,vel,acc] = trajectory.getState(t);
    poscmdlog = [poscmdlog; pos];
    posfbklog = [posfbklog; fbk.position];
    
    % Account for external efforts due to the gas spring
    effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
    
    % Calculate commanded efforts to assist with tracking the trajectory.
    % gravCompEfforts() uses knowledge of the arm's kinematics and mass to
    % compensate for the weight of the arm.  dynamicCompEfforts() uses the
    % kinematics and mass to compensate for the commanded accelerations of
    % the arm.
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
    
    % Fill in the CommandStruct and send commands to the arm
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


% Go to the other points a little bit more quickly.  
timeToMove = 1.5;             % [sec]
time = [ 0 timeToMove ];    % [sec]

group.send(CommandStruct());
% Go to all the different points.  Calculate new point-to-point
% trajectories one at a time.
for i=1:length(xyzTargets)+1
    
    % Shift to the next target position
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];

    % Get the trajectory to the next target         
    trajectory = trajGen.newJointMove( waypoints, 'time', time );
%     trajectory = trajGen.newLinearMove( waypoints, 'time', time );

    
    % Get feedback and update the timer
    t0 = fbk.time;
    t = 0;
    
    % Execute the motion to go to the next target
    while t < trajectory.getDuration

        % Get feedback and update the timer
        fbk = group.getNextFeedback();
        t = fbk.time - t0;
        
%         % collision detection
%         if abs(fbk.effort(5) - fbk.effortCmd(5)) > 0.3
%             group.send(CommandStruct());
%             break
%         end
        
        % Get new commands from the trajectory
        [pos,vel,acc] = trajectory.getState(t);
        poscmdlog = [poscmdlog; pos];
        posfbklog = [posfbklog; fbk.position];

        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        % Calculate commanded efforts to assist with tracking the trajectory.
        % gravCompEfforts() uses knowledge of the arm's kinematics and mass to
        % compensate for the weight of the arm.  dynamicCompEfforts() uses the
        % kinematics and mass to compensate for the commanded accelerations of
        % the arm.
        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);
                                                              
        % Fill in the CommandStruct and send commands to the arm    
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
        
        if isIO == 1
            fbkIO = IOgroup.getNextFeedbackIO();
        
            grippercmd.position = [];
            grippercmd.velocity = [];
            grippercmd.effort = -2 + fbkIO.a6*(-3);
        
            grippergroup.send(grippercmd);
        else
            grippercmd.position = [];
            grippercmd.velocity = [];
            grippercmd.effort = -6 * mod(i,2) +1;
        
            grippergroup.send(grippercmd);      
        end
    end
    
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();
HebiUtils.plotLogs( log, 'position');
HebiUtils.plotLogs( log, 'velocity');
HebiUtils.plotLogs( log, 'effort');