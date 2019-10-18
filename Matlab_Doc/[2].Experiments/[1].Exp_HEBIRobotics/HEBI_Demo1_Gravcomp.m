% 헤비 예제 수정 파일
% hold spring 활용해서, uncertainty 등에 의한 drift 방지
%%
clear all;
close all;
clc

%% setting parmaeters
%holdspring 1 : drift를 kp 제어로 제거
%holdspring 2 : drit 무시
holdspring = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 2;

%% HEBI setting
HebiLookup.initialize();

% % kin = HebiKinematics('hrdf/6-DoF_arm_w_gripper_KIMM.hrdf');
% kin = setupArm('6dof_w_gripper');
% gains = HebiUtils.loadGains('gains/6-DoF_arm_gains_KIMM[basic].xml');
% trajGen = HebiTrajectoryGenerator();
% 
% familyName = 'Arm';%'6-DoF Arm';
% moduleNames = {'Base','Shoulder','Elbow','Wrist1','Wrist2','Wrist3'};
% 
% group = HebiLookup.newGroupFromNames( familyName, moduleNames );
% group.send('gains',gains);
% cmd = CommandStruct();
[kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;

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
% Velocity threshold below which robot is considered moving. Instead of
% trying to determine movement based on sensor data, it could also be a
% physical button on the robot that users would need to press before being
% able to move the robot.
velocityThreshold = 0.3;

% Stiffness (like Kp gain) of the virtual spring. i.e., how hard should  
% it try to keep the position. Can be different for each module (vector)
% and may need some tuning. A stiffness of all zeros effectively disables 
% the spring.
stiffness = 10 * ones(1,kin.getNumDoF());

fbk = group.getNextFeedback();
idlePos = fbk.position;
stiffness = stiffness .* ones(1,group.getNumModules()); % turn into vector

while true
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
    pause(0.001);
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();
HebiUtils.plotLogs( log, 'position', 'figNum', 101 );
HebiUtils.plotLogs( log, 'velocity', 'figNum', 102 );
HebiUtils.plotLogs( log, 'effort', 'figNum', 103 );