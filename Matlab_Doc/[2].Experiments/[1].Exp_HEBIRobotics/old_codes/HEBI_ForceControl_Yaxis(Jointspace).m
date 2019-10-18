% Y방향 (joint space) 힘제어 구현
% 아직 힘제어를 잘 이해하지 못했을 당시의 파일임
% y방향만 힘제어 하고, 나머지 방향은 p게인 활용한 holding torque를 활용
%개념적으로 하이브리드 힘/위치 제어와 유사하나 joint space에서 구현됨(작업 공간에서의 활용이 어려움)
%%
clear *;
close all;

%% setting parmaeters
%ctrmode 1 : 모터에 position 지령(중력,coriolis등은 토크 지령으로)
%ctrmode 2 : 모터에 torque 지령
ctrmode = 2;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

%% Target Waypoints
% xyzTargets = [ 0.50   0.50   0.50   0.50;    % x [m]
%                0.10   0.10   0.10   0.10;    % y [m]
%               -0.00  -0.20  -0.00  -0.20];  % z [m]
% xyzTargets = [ 0.50   0.50   0.50   0.50;    % x [m]
%                0.10   0.10   0.10   0.10;    % y [m]
%               -0.00  -0.00  -0.00  -0.00];  % z [m]
% xyzTargets = [ 0.50;    % x [m]
%                0.10;    % y [m]
%               -0.10];  % z [m]
% xyzTargets = [ 0.38   0.38   0.38   0.38;    % x [m]
%                0.07   0.07   0.07   0.07;    % y [m]
%               -0.12  -0.33  -0.12  -0.33];  % z [m]
% xyzTargets = [ 0.38   0.38   0.38   0.38;    % x [m]
%                0.07   0.07   0.07   0.07;    % y [m]
%               -0.12  -0.12  -0.12  -0.12];  % z [m]
xyzTargets = [ 0.38   0.38;    % x [m]
               0.17  -0.17;    % y [m]
              -0.25  -0.25];  % z [m]
          
desired_deflection = 0.005;

           
           
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

group.startLog('dir','logs');
           
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
% rotMatTarget = R_y(pi);   % [3x3 SO3 Matrix]
rotMatTarget = R_x(pi);   % [3x3 SO3 Matrix]
              
initPosition = [ 0 pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]

% Do IK to get joint position waypoints for each XYZ target, as well as the 
% desired orientation of the end effector.  Copy the first waypoint at the 
% end, so that it closes the loop
for i=1:length(xyzTargets(1,:))
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                 'SO3', rotMatTarget, ...
                                 'initial', initPosition ); 
end
% posTargets(end+1,:) = posTargets(end,:);

% Get the initial feedback joint positions, and go from there to the first
% waypoint, using the trajectory API.  
fbk = group.getNextFeedback();
waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
timeToMove = 3;             % [sec]
time = [ 0 timeToMove ];    % [sec]

% Calculate initial trajectory to starting posiiton
trajectory = trajGen.newJointMove( waypoints, 'time', time );

% Initialize timer
t0 = fbk.time;
t = 0;

poscmdlog = [];
posfbklog = [];
jointTorquelog = [];
Felog = [];
deflectionlog = [];
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
timeToMove = 5;             % [sec]
time = [ 0 timeToMove ];    % [sec]

group.send(CommandStruct());
% Go to all the different points.  Calculate new point-to-point
% trajectories one at a time.
flag = 0;

velocityThreshold = 1;
stiffness = 10 * ones(1,kin.getNumDoF());
fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

for i=1:size(xyzTargets,2)-1
    
    % Shift to the next target position
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];

    % Get the trajectory to the next target         
    trajectory = trajGen.newJointMove( waypoints, 'time', time );
%     trajectory = trajGen.newLinearMove( waypoints, 'time', time );

    
    % Get feedback and update the timer
    t0 = fbk.time;
    t = 0;
    
    deflection_pre = 0;
    % Execute the motion to go to the next target
    while t < trajectory.getDuration
        
        % Get feedback and update the timer
        fbk = group.getNextFeedbackFull();
        t = fbk.time - t0;
        
        %get Jacobian
        J = kin.getJacobian('endeffector',fbk.position);
        
        % deflection & joint torque & endeffector force
        deflectionlog = [deflectionlog;fbk.deflection];   
        jointTorque = fbk.deflection'.*[130 170 70 70 70 70]';
        Fe = inv(J')*jointTorque;
        jointTorquelog = [jointTorquelog;jointTorque'];
        Felog = [Felog;Fe'];    
        
        % collision detection
%         if i >= 1
% %             if (abs(fbk.effort(2) - fbk.effortCmd(2)) > 0.3) || (abs(fbk.effort(3) - fbk.effortCmd(3)) > 0.3) || (abs(fbk.effort(4) - fbk.effortCmd(4)) > 0.3) 
%             if fbk.deflection(1) > 0.005   
%                 group.send(CommandStruct());
%                 break
%             end
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
        
        if flag == 0
            if fbk.deflection(1) >= desired_deflection
                flag = 1;
            end
        end
        
        if flag == 0
            if ctrmode == 1
                cmd.position = pos;
                cmd.velocity = vel;
                cmd.effort = dynamicCompEfforts + gravCompEfforts + effortOffset;
            elseif ctrmode == 2
                cmdeffort = -gains.positionKp.*(fbk.position - pos)  -gains.velocityKp.*(fbk.velocity - vel) + ...
                            dynamicCompEfforts + gravCompEfforts + effortOffset;
                cmd.position = [];
                cmd.velocity = [];
                cmd.effort = cmdeffort;
            end
        end
        
        if flag == 1
            cmdeffort = gravCompEfforts + effortOffset;
            
            % Find whether robot is actively moving
            isMoving = max(abs(fbk.velocity)) > velocityThreshold;
            if isMoving
                % Update idle position
                idlePos = fbk.position;
            else
                % Add efforts from virtual spring to maintain position
                driftError = idlePos - fbk.position;
                holdingEffort = driftError .* stiffness;
                holdingEffort(1) = 0;
                cmdeffort = cmdeffort + holdingEffort;
            end  
            
            cmdeffort(1) = cmdeffort(1) + 100 * (fbk.deflection(1) - desired_deflection);%...
                                        %+  100 * (fbk.deflection(1) - deflection_pre);
            cmd.effort = cmdeffort;
        end
        
        group.send(cmd);
        
        deflection_pre = fbk.deflection(1);
    end
    
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();
% HebiUtils.plotLogs( log, 'position');
% HebiUtils.plotLogs( log, 'velocity');
HebiUtils.plotLogs( log, 'effort');

figure;plot(deflectionlog(:,1),'LineWidth',1.5)
hold on
plot([0 500],[desired_deflection desired_deflection],'--r')
title('position+force ocntrol')
legend('deflection','threshold')
ylabel('deflection(rad)')
xlabel('step')
ylim([-0.008 0.01])
grid on

figure;
subplot(2,3,1)
plot(jointTorquelog(:,1))
title('jointTorque')
subplot(2,3,2)
plot(jointTorquelog(:,2))
subplot(2,3,3)
plot(jointTorquelog(:,3))
subplot(2,3,4)
plot(jointTorquelog(:,4))
subplot(2,3,5)
plot(jointTorquelog(:,5))
subplot(2,3,6)
plot(jointTorquelog(:,6))


figure;
subplot(2,3,1)
plot(Felog(:,1))
title('Endeffector Force')
subplot(2,3,2)
plot(Felog(:,2))
subplot(2,3,3)
plot(Felog(:,3))
subplot(2,3,4)
plot(Felog(:,4))
subplot(2,3,5)
plot(Felog(:,5))
subplot(2,3,6)
plot(Felog(:,6))
