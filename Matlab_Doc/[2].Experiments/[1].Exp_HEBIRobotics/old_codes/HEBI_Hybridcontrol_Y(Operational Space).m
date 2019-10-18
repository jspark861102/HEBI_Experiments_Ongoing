%하이브리드 힘/위치 제어를 처음으로 구현한 파일임
%z축은 가스 스프링 때문에 문제가 왜곡되서 보여서 Y축을 먼저구현해봄
%완결판은 데모 파일임
%%
clear *;
close all;
clc

%% setting parmaeters
%controlmode = 1 : cartesian space control
%controlmode = 2 : joint pace control
controlmode = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

% position/force control Threshold
desired_force = 2;

% holdeffect velocity Threshold
velocityThreshold = 1;

%% HEBI setting
HebiLookup.initialize();
[kin,gains,trajGen,group,cmd] = HEBI_Arm_Initialize;
group.startLog('dir','logs');
           
%% Target Waypoints
xyzTargets = [ 0.42   0.42;    % x [m]
               0.07  -0.07;    % y [m]
              -0.20  -0.20];  % z [m]

% baseframe을 기준으로 endeffector의 회전 방향          
% rotMatTarget = R_y(pi/2);   % [3x3 SO3 Matrix]
% rotMatTarget = R_y(pi);   % [3x3 SO3 Matrix]
rotMatTarget = R_x(pi);   % [3x3 SO3 Matrix]


% Inverse Kinematics initial position
initPosition = [ 0 pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]

for i=1:length(xyzTargets(1,:))
    posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                 'SO3', rotMatTarget, ...
                                 'initial', initPosition ); 
end           

%% gravity direction
[gravityVec] = HEBI_Arm_gravity(gravitysetting);

%% holdeffect setting
stiffness = 10 * ones(1,kin.getNumDoF());

%% log data setting
poscmdlog = [];posfbklog = [];
Telog = [];Felog = [];deflectionlog = [];Tc_poslog = [];Fc_poslog = [];Tc_forcelog = [];Fc_forcelog = [];Tmlog = [];
ControlSwitchlog = []; Xelog = []; Velog = []; Xdlog = []; Vdlog = [];

%% trajectory & control
%%%%%%%%%%%%%%%%%%%%% go from here to initial waypoint %%%%%%%%%%%%%%%%%%%%
%control setting
fbk = group.getNextFeedback();
waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
time = [ 0 3];    % [sec]
trajectory = trajGen.newJointMove( waypoints, 'time', time );
t0 = fbk.time;
t = 0;
ControlSwitch = 0;
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedbackFull();
    t = fbk.time - t0;
    [pos,vel,acc] = trajectory.getState(t);
    
    %get Jacobian
    J = kin.getJacobian('endeffector',fbk.position);

    %get external torque exerted on joint and endeffector
    Te = fbk.deflection'.*[130 170 70 70 70 70]';
    Fe = inv(J')*Te;             
    
    % Account for external efforts due to the gas spring
    effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
    
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );
                                                
    Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
    Tc = -gains.positionKp.*(fbk.position - pos);% -gains.velocityKp.*(fbk.velocity - vel);
    
    cmd.position = [];
    cmd.velocity = [];
    cmd.effort = Tm + Tc;
    
    group.send(cmd);    
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% go next waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%control setting
time = [ 0 7;0 3];    % [sec]
idlePos = fbk.position;
Xd = [xyzTargets(:,1);pi;0;0]';
Xe = Xd; %이부분이 문제다 FK에서 좌표값이 구할 수 없어 임의로 넣는 값. 
for i=1:size(xyzTargets,2)-1
    
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];
    trajectory = trajGen.newJointMove( waypoints, 'time', time(i,:) );
    
    t0 = fbk.time;
    t = 0;    
    ControlSwitch = 0;    deflection_pre = 0;
    while t < trajectory.getDuration
        
        % Get feedback and update the timer
        fbk = group.getNextFeedbackFull();
        t = fbk.time - t0;
        [pos,vel,acc] = trajectory.getState(t);
        
        %get Jacobian
        J = kin.getJacobian('endeffector',fbk.position);
        
        %get endeffector position%velocity
        dt = 1/group.getFeedbackFrequency;
        Ve = (J * fbk.velocity')';
        Xe = Xe + Ve * dt;
        
        %get external torque exerted on joint and endeffector
        Te = fbk.deflection.*[130 170 70 70 70 70];
        Fe = (inv(J')*Te')';             
      
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);
                                                    
        %get endeffector desired position%velocity
        Jd = kin.getJacobian('endeffector',pos);
        Vd = (Jd * vel')';
        Xd = Xd + Vd * dt;
        
        %힘제어 작동 여부                                                       
        if ControlSwitch == 0
            if Fe(2) >= desired_force
                ControlSwitch = 1
            end
        end
        
        ePgain = [300 300 600 20 20 10];
%         ePgain = [100 100 250 20 20 10];
        eVgain = [0.1 0.1 0.1 0.1 0.1 0.1];        
        %위치제어
        Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
        if controlmode == 1 %cartesian sapce control           
            Fc_pos = -ePgain .* (Xe - Xd) - eVgain .* (Ve - Vd); 
            if ControlSwitch == 1
                Fc_pos(2) = 0;
                here = 1
            end
            Tc_pos = (J' * Fc_pos')';
        else %joint space control            
            Tc_pos = -gains.positionKp.*(fbk.position - pos)  -gains.velocityKp.*(fbk.velocity - vel);
        end
        
        %힘제어
        if ControlSwitch == 1            
            Fc_force = [0 0.3*(Fe(2) - desired_force) 0 0 0 0];
            Tc_force = (J' * Fc_force')';
        else
            Fc_force = [0 0 0 0 0 0];
            Tc_force = (J' * Fc_force')';
        end

        cmd.position = [];
        cmd.velocity = [];
        cmd.effort = Tm + Tc_pos + Tc_force;        
        group.send(cmd);    
        
        % data log
        poscmdlog = [poscmdlog; pos];
        posfbklog = [posfbklog; fbk.position];    
        deflectionlog = [deflectionlog;fbk.deflection];
        Telog = [Telog;Te];
        Felog = [Felog;Fe];   
        Tmlog = [Tmlog;Tm];
        Tc_poslog = [Tc_poslog;Tc_pos];
        Fc_poslog = [Fc_poslog;Fc_pos];
        Tc_forcelog = [Tc_forcelog;Tc_force];
        Fc_forcelog = [Fc_forcelog;Fc_force];
        Xelog = [Xelog;Xe];
        Velog = [Velog;Ve];
        Xdlog = [Xdlog;Xd];
        Vdlog = [Vdlog;Vd];       
        ControlSwitchlog = [ControlSwitchlog;ControlSwitch];          
    end    
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
log = group.stopLog();

% HebiUtils.plotLogs( log, 'position');
% HebiUtils.plotLogs( log, 'velocity');
% HebiUtils.plotLogs( log, 'effort');

% subplot6a([0:dt:(size(Telog,1)-1)*dt],Telog,'Te')    
subplot6a([0:dt:(size(Felog,1)-1)*dt],Felog,'Fe')  

% subplot6a([0:dt:(size(Tmlog,1)-1)*dt],Tmlog,'Tm')

subplot6a([0:dt:(size(Tc_poslog,1)-1)*dt],Tc_poslog,'Tc pos') 
subplot6a([0:dt:(size(Fc_poslog,1)-1)*dt],Fc_poslog,'Fc pos') 
subplot6a([0:dt:(size(Tc_forcelog,1)-1)*dt],Tc_forcelog,'Tc force') 
subplot6a([0:dt:(size(Fc_forcelog,1)-1)*dt],Fc_forcelog,'Fc force') 

subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],posfbklog-poscmdlog,'error')

% subplot6a([0:dt:(size(Xelog,1)-1)*dt],Xelog,'Xe') 
% subplot6a([0:dt:(size(Velog,1)-1)*dt],Velog,'Ve') 
% subplot6a([0:dt:(size(Xdlog,1)-1)*dt],Xdlog,'Xd') 
% subplot6a([0:dt:(size(Vdlog,1)-1)*dt],Vdlog,'Vd') 
