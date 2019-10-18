clear all
close all
clc 

%% HEBI setting
HebiLookup.initialize();

% kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper_KIMM.hrdf');
kin = setupArm('6dof_w_gripper');
gains = HebiUtils.loadGains('gains/6-DoF_arm_gains_KIMM[basic].xml');
% % gains = HebiUtils.loadGains('gains/6-DoF_arm_gains_KIMM[impedance].xml');
trajGen = HebiTrajectoryGenerator();

familyName = 'Arm';%'6-DoF Arm';
moduleNames = {'Base','Shoulder','Elbow','Wrist1','Wrist2','Wrist3'};

group = HebiLookup.newGroupFromNames( familyName, moduleNames );
group.send('gains',gains);
cmd = CommandStruct();

%% Setup
group.setFeedbackFrequency(200);
numDoF = group.getNumModules();

    % Impendance Control Gains
    % NOTE: The gains corespond to:
    % [ trans_x trans_y trans_z rot_x rot_y rot_z ]
    %
    % Translations and Rotations can be specified in the
    % base frame or in the end effector frame.  See code below for
    % details.
    %
    % GAINS ARE IN THE END-EFFECTOR FRAME

%     % HOLD POSITION ONLY (Allow rotation around trochar point)
%     gainsInEndEffectorFrame = true;
%     damperGains = [20; 20; 20; .1; .1; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 500; 0; 0; 0];  % (N/m) or (Nm/rad)
 
    % HOLD POSITION AND TIP VECTOR
%     gainsInEndEffectorFrame = true;
%     damperGains = [10; 10; 10; .1; .1; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 500; 50; 50; 0];  % (N/m) or (Nm/rad)
 
%     % HOLD POSITION AND TIP VECTOR - ALLOW MOTION ALONG TROCHAR Z-AXIS
%     gainsInEndEffectorFrame = true;
%     damperGains = [10; 10; 0; .1; .1; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
%     springGains = [500; 500; 0; 50; 50; 0];  % (N/m) or (Nm/rad)

%     % HOLD POSITION AND TIP VECTOR - ALLOW MOTION ALONG TROCHAR X-AXIS
    gainsInEndEffectorFrame = true;
    damperGains = [0; 10; 10; .1; .1; .0;]; % (N/(m/sec)) or (Nm/(rad/sec))
    springGains = [0; 500;  500; 50; 50; 0];  % (N/m) or (Nm/rad)

% Keyboard input
kb = HebiKeyboard();

% Select whether waypoints should be done as a single trajectory, or
% multiple trajectories that stop in between.
stopBetweenWaypoints = true;

% Select whether you want to log and visualize the replay movement
enableLogging = true;

% Velocity threshold below which robot is considered moving. Instead of
% trying to determine movement based on sensor data, it could also be a
% physical button on the robot that users would need to press before being
% able to move the robot.
velocityThreshold = 0.5;

% Stiffness (like Kp gain) of the virtual spring. i.e., how hard should  
% it try to keep the position. Can be different for each module (vector)
% and may need some tuning. A stiffness of all zeros effectively disables 
% the spring.
stiffness = 10 * ones(1,kin.getNumDoF());

waypoints = [];
keys = read(kb);
prevKeys = keys;

% Start background logging 
if enableLogging
   logFile = group.startLog('dir','logs');
end
    
fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

while true

    if keys.ESC
        break;
    end
    
    pause(0.15); 
    
    keys = read(kb);
    disp('Move the arm to desired position.');
    disp('   - Press SPACE to lock trochar point.');
    disp('   - Press ESC to quit.');
    
    while keys.SPACE == 0 && keys.ESC == 0

        keys = read(kb);
        
        % Do grav-comp while training waypoints
        fbk = group.getNextFeedbackFull();
        baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                              fbk.orientationX(1), ...
                                              fbk.orientationY(1), ...
                                              fbk.orientationZ(1) ] );
        gravityVec = -baseRotMat(3,1:3);
        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
            
        cmd.effort = kin.getGravCompEfforts(fbk.position, gravityVec) ...
            + effortOffset;
        
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
        
        group.send(cmd);
    end
    
    if keys.ESC
        break;
    end
    
    % Get the current location of the end effector
    armTipFK = kin.getFK('endeffector',fbk.position);
    trocharXYZ = armTipFK(1:3,4);
    trocharRotMat = armTipFK(1:3,1:3);
    
    % Velocity commands
    armCmdJointAngs = fbk.position;
    armCmdJointVels = zeros(1,numDoF);

    %% Hold the trochar point
    disp('Holding tochar point...');
    disp('   - Press SPACE to reset to a new point.');
    disp('   - Press ESC to quit.');

    pause(0.1); 
    keys = read(kb);
    
    while keys.SPACE == 0 && keys.ESC == 0
        
        keys = read(kb);
        fbk = group.getNextFeedbackFull();
        
        armTipFK = kin.getFK('endeffector',fbk.position);
        J_armTip = kin.getJacobian('endeffector',fbk.position);
        
        % Calculate Impedence Control Wrenches and Appropraite Joint Torque
        springWrench = zeros(6,1);
        damperWrench = zeros(6,1);
        
        % Linear error is easy
        xyzError = trocharXYZ - armTipFK(1:3,4);
        
        % Rotational error involves calculating axis-angle from the
        % resulting error in S03 and providing a torque around that axis.
        errorRotMat = trocharRotMat * armTipFK(1:3,1:3)';
        [axis, angle] = HebiUtils.rotMat2axAng( errorRotMat );
        rotErrorVec = angle * axis;
        
        if gainsInEndEffectorFrame
            xyzError = armTipFK(1:3,1:3)' * xyzError;
            rotErrorVec = armTipFK(1:3,1:3)' * rotErrorVec;
        end
        
        posError = [xyzError; rotErrorVec];
        velError = J_armTip * (armCmdJointVels - fbk.velocity)';     

        springWrench(1:3) = springGains(1:3) .* posError(1:3); % linear force
        springWrench(4:6) = springGains(4:6) .* posError(4:6); % rotational torque
        
        if gainsInEndEffectorFrame
            springWrench(1:3) = armTipFK(1:3,1:3) * springWrench(1:3);
            springWrench(4:6) = armTipFK(1:3,1:3) * springWrench(4:6);
        end
        
        damperWrench(1:3) = damperGains(1:3) .* velError(1:3); % linear damping
        damperWrench(4:6) = damperGains(4:6) .* velError(4:6); % rotational damping

        impedanceTorque = J_armTip' * (springWrench + damperWrench); 

        % Gravity Compensation Torques
        baseRotMat = HebiUtils.quat2rotMat( [ fbk.orientationW(1), ...
                                              fbk.orientationX(1), ...
                                              fbk.orientationY(1), ...
                                              fbk.orientationZ(1) ] );
        gravityVec = -baseRotMat(3,1:3);
        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
            
        gravCompTorque = kin.getGravCompEfforts(fbk.position, gravityVec);
        
        % cmd.position = armCmdJointAngs;
        cmd.velocity = armCmdJointVels;
        cmd.effort = impedanceTorque' + gravCompTorque + effortOffset;
      
        group.send(cmd);
    end
    
end

% Stop background logging and visualize
if enableLogging
    disp('Quitting. Loading and plotting log data...');
    hebilog = group.stopLogFull();
    HebiUtils.plotLogs(hebilog, 'position', 'figNum', 101);
    HebiUtils.plotLogs(hebilog, 'velocity', 'figNum', 102);
    HebiUtils.plotLogs(hebilog, 'effort', 'figNum', 103);
else
    disp('Quitting. No Logging...');
end

disp('All done.');
