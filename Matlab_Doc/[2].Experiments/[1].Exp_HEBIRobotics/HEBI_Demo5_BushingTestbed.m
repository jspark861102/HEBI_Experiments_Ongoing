%%
clear *;
close all;
clc

%% setting parmaeters
%11:pick&insert
%12:pick&pushing&pivoting&place
%13:pick&pushing&insert
demo_case = 14;

%controlmode = 1 : cartesian space control
%controlmode = 2 : joint pace control
controlmode = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

% position/force control Threshold
desired_force = [1000 -11 1000 1000 1000 1000]; %������ ���� �ϰ� ���� ���� ���� ���� ũ�� ��������

% force control smooting
smoothing_duration = 0.2; % sec

% holdeffect velocity Threshold
velocityThreshold = 1;

%% HEBI setting
HebiLookup.initialize();
[kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;
% group.startLog('dir','logs');
           
%% Target Waypoints      
% Inverse Kinematics initial position
initPosition_front = [ 0   pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]
initPosition_back =  [ -pi pi/4 pi/2 pi/4 -pi pi/2 ];  % [rad]
[posTargets, xyzTargets, rotMatTarget, control_time, gripperforce, FT_trigger] = TargetWaypoints_BushingTestbed(demo_case, kin, initPosition_front, initPosition_back);
           
%% gravity direction
[gravityVec] = HEBI_Arm_gravity(gravitysetting);

%% holdeffect setting
stiffness = 10 * ones(1,kin.getNumDoF());

%% log data setting
poscmdlog = [];posfbklog = [];
Telog = [];Felog = [];deflectionlog = [];Tc_poslog = [];Fc_poslog = [];Tc_forcelog = [];Fc_forcelog = [];Tmlog = [];
ControlSwitchlog = []; Xelog = []; Velog = []; Xdlog = []; Vdlog = [];T_gripperlog = [];smoothing_factorlog = [];Fmlog = [];

%% trajectory & control
%%%%%%%%%%%%%%%%%%%%% go from here to first waypoint %%%%%%%%%%%%%%%%%%%%
%control setting
fbk = group.getNextFeedbackFull(); %�ʱ��ڼ� ����
fbk_gripper = grippergroup.getNextFeedbackFull();

waypoints = [ fbk.position;
              posTargets(1,:) ];    % [rad]
trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(1)] );
t0 = fbk.time;
t = 0;
while t < trajectory.getDuration
    
    % Get feedback and update the timer
    fbk = group.getNextFeedbackFull();
    fbk_gripper = grippergroup.getNextFeedbackFull(); 

    t = fbk.time - t0;
    [pos,vel,acc] = trajectory.getState(t);              
    
    % Account for external efforts due to the gas spring
    effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];
    
    gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
    dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                    pos, vel, acc );                                                
    Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;
    
    cmd.position = pos;
    cmd.velocity = [];
    cmd.effort = Tm;    
    group.send(cmd);    
    
    % �׸��� ����        
    grippercmd.position = [];
    grippercmd.velocity = [];
    grippercmd.effort = gripperforce(1);    
    grippergroup.send(grippercmd);  
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% go next waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial condition
ControlSwitch = [0 0 0 0 0 0]; % �ʱⰪ, ��ġ����� ����
Xe = [xyzTargets(:,1);pi;0;0]'; %Xe �ʱⰪ�� ���� ������ �Ϻ��� ���� �ƴٴ� �����Ͽ� ����
Xd = [xyzTargets(:,1);pi;0;0]'; %cartesian space control�� ���� xd�ʱⰪ
real_dt_set=[];
% for i=1:size(xyzTargets,2)-1     
for i=1:size(posTargets,1)-1     

    if any(ControlSwitch) %���� while������ ����� �۵��Ǿ��ٸ�
        waypoints = [ fbk.position ;
                      posTargets(i+1,:) ];
    else        
        waypoints = [ posTargets(i,:) ;
                      posTargets(i+1,:) ];
    end
  
    
    trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    t0 = fbk.time;
    t = 0; pre_t = 0; 
    ControlSwitch = [0 0 0 0 0 0]; % ���� target���� �̵��� ���� �׻� ��ġ����� ����  
    smoothing_factor = 1;
    is_newwaypoint = 0;
    while t < trajectory.getDuration
        
        % Get feedback and update the timer
        fbk = group.getNextFeedbackFull();
        fbk_gripper = grippergroup.getNextFeedbackFull(); 
        t = fbk.time - t0;
        real_dt = t - pre_t;
        pre_t = t;
        real_dt_set = [real_dt_set real_dt];
        
        %get Jacobian
        J = kin.getJacobian('endeffector',fbk.position);
        
        %get endeffector position%velocity
        %������ joint position���κ��� cartesian position�� �ٷ� �˼������� ������ API�� ����.
        dt = 1/group.getFeedbackFrequency;
        Ve = (J * fbk.velocity')';
        Xe = Xe + Ve * dt;
        
        %get external torque exerted on joint and endeffector
        Te = fbk.deflection.*[70 170 70 70 70 70];
        Fe = (inv(J')*Te')';       
        
        if FT_trigger(i+1) ~= 0
            for i_switch = 1 : length(Fe) %6�࿡ ���� Ȯ��
                if ControlSwitch(i_switch) == 0 %���� ��ġ ������ ��쿡�� ������� ����
                    if desired_force(i_switch) > 0 %desired force�� 0���� ū ��� 
                        if Fe(i_switch) >= desired_force(i_switch)
                           is_newwaypoint = 1;
                           ControlSwitch(i_switch) = 1
                           i_switch = i_switch
                        end
                    elseif desired_force(i_switch) < 0 %desired force�� 0���� ���� ��� 
                        if Fe(i_switch) <= desired_force(i_switch)
                           is_newwaypoint = 1;
                           ControlSwitch(i_switch) = 1
                           i_switch = i_switch
                        end
                    end                        
                end
            end            
            if trajectory.getDuration - t <= smoothing_duration
                smoothing_factor = 1/smoothing_duration * (trajectory.getDuration - t);
%                 smoothing_factor = 1;   
            end            
        end
        
        if is_newwaypoint == 1
%             newTarget = xyzTargets(:,i+1) .* ([1 1 1] - ControlSwitch(1:3))' + (Xe(1:3) .* ControlSwitch(1:3))'; 
%             [Xe(1:3)' xyzTargets(:,i+1) newTarget]
%             posTargets_FT = kin.getIK( 'xyz', newTarget, ...
%                                        'SO3', R_x(pi), ...
%                                        'initial', initPosition );                                     
            newwaypoint = [ fbk.position ;
                          fbk.position ];
            waypoints;
            newwaypoint;
%             posTargets_FT

            trajectory = trajGen.newJointMove( newwaypoint, 'time', [0 control_time(i+1)]);
            trajectory_m = trajectory;
            is_newwaypoint = 0; % �� ������ ������� ����Ǿ��� �� �ѹ��� ����
        end         
        
        [pos,vel,acc] = trajectory.getState(t);

        
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);   
        Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;   
        
        Fc_force = (([0 2.0 0 0 0 0].*(Fe - desired_force.*[1 smoothing_factor 1 1 1 1])) - Fe).*ControlSwitch; 
        Tc_force = (J' * Fc_force')';% - gains.positionKp.*(fbk.position - pos)*any(ControlSwitch);
               
        if any(ControlSwitch) %�������� ���      
            Xd = Xd .* ([1 1 1 1 1 1] - ControlSwitch) + Xe .* ControlSwitch; % ������� ��ġ����� ����ġ �ɶ� Ƣ�� ���� ������, ������� Xe->Xd�� �������� �����Ƿ�, ���� ��ġ�� Xd�� �ٸ� ���� �����Ƿ� ���� �ʿ�
            Vd = Vd .* ([1 1 1 1 1 1] - ControlSwitch) + Ve .* ControlSwitch; % ������� ��ġ����� ����ġ �ɶ� Ƣ�� ���� ������
        else %��ġ������ ���
            Jd = kin.getJacobian('endeffector',pos); %pos�� feedback pos�ƴ�, desired pos
            Vd = (Jd * vel')';
            Xd = Xd + Vd * dt;
        end   
        ePgain = [100 100 150 9 9 6];
%         ePgain = [100 100 5 9 9 6];
        eVgain = [0.1 0.1 0.1 0.1 0.1 0.1]; 
        Fc_pos = -ePgain .* (Xe - (Xd + [0 0 0.00 0 0 0])) - eVgain .* (Ve - Vd); % ���Ƿ� Xd �ٲ� ���� ���� �ʿ�
        Fc_pos = Fc_pos .* ([1 1 1 1 1 1] - ControlSwitch); %������ ���� ��ġ��� 0���� ����
        Tc_pos = (J' * Fc_pos')'*any(ControlSwitch);      
%         Tc_pos = (J' * Fc_pos')'*any(FT_trigger(i+1));        

        
        cmd.position = pos;
        cmd.velocity = [];
        cmd.effort = Tm + Tc_force + Tc_pos;        
        group.send(cmd);   
        
        % �׸��� ����        
        grippercmd.position = [];
        grippercmd.velocity = [];
        grippercmd.effort = gripperforce(i+1); 
        grippergroup.send(grippercmd); 
        
        
        % data log
        poscmdlog = [poscmdlog; pos];
        posfbklog = [posfbklog; fbk.position];    
        deflectionlog = [deflectionlog;fbk.deflection];
        Telog = [Telog;Te];
        Felog = [Felog;Fe];   
        Tmlog = [Tmlog;Tm];
        Tc_poslog = [Tc_poslog;Tc_pos];
        if controlmode == 1
            Fc_poslog = [Fc_poslog;Fc_pos];
        end
        Tc_forcelog = [Tc_forcelog;Tc_force];
        Fc_forcelog = [Fc_forcelog;Fc_force];
        Xelog = [Xelog;Xe];
        Velog = [Velog;Ve];
        Xdlog = [Xdlog;Xd];
        Vdlog = [Vdlog;Vd];       
        ControlSwitchlog = [ControlSwitchlog;ControlSwitch];    
%         T_gripperlog = [T_gripperlog;T_gripper];
        smoothing_factorlog = [smoothing_factorlog;smoothing_factor];
%         Fmlog = [Fmlog;Fm];

    end    
end

%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
% log = group.stopLog();

% HebiUtils.plotLogs( log, 'position');
% HebiUtils.plotLogs( log, 'velocity');
% HebiUtils.plotLogs( log, 'effort');

% subplot6a([0:dt:(size(Telog,1)-1)*dt],Telog,'Te',control_time)    
% subplot6a([0:dt:(size(Felog,1)-1)*dt],Felog,'Fe',control_time)  

figure;plot([0:dt:(size(Felog,1)-1)*dt],Felog(:,2))  
grid on
title('contact force')
xlabel('time')
ylabel('force(N)')
set(gcf, 'Position', [10 40 1500 700])


% % % % subplot6a([0:dt:(size(Tmlog,1)-1)*dt],Tmlog,'Tm',control_time)
% % % 
% % % subplot6a([0:dt:(size(Tc_poslog,1)-1)*dt],Tc_poslog,'Tc pos',control_time) 
% % % subplot6a([0:dt:(size(Fc_poslog,1)-1)*dt],Fc_poslog,'Fc pos',control_time) 
% % % subplot6a([0:dt:(size(Tc_forcelog,1)-1)*dt],Tc_forcelog,'Tc force',control_time) 
% % % subplot6a([0:dt:(size(Fc_forcelog,1)-1)*dt],Fc_forcelog,'Fc force',control_time) 
% % % subplot6a([0:dt:(size(Tmlog,1)-1)*dt],Tmlog + Tc_poslog + Tc_forcelog,'Tc',control_time) 
% % % 
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],posfbklog-poscmdlog,'poserror',control_time)
% % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],Xelog-Xdlog,'Xerror',control_time)
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],Velog-Vdlog,'Verror',control_time)
% % % 
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],posfbklog,'posfbk',control_time)
% % % % subplot6a([0:dt:(size(poscmdlog,1)-1)*dt],poscmdlog,'poscmd',control_time)
% % % 
% % % % subplot6a([0:dt:(size(Xelog,1)-1)*dt],Xelog,'Xe',control_time) 
% % % % subplot6a([0:dt:(size(Velog,1)-1)*dt],Velog,'Ve',control_time) 
% % % % subplot6a([0:dt:(size(Xdlog,1)-1)*dt],Xdlog,'Xd',control_time) 
% % % % subplot6a([0:dt:(size(Vdlog,1)-1)*dt],Vdlog,'Vd',control_time) 
% % % 
% % % % subplot6a([0:dt:(size(deflectionlog,1)-1)*dt],deflectionlog,'defelction',control_time) 
% % % 
% % % % figure;plot([0:dt:(size(T_gripperlog,1)-1)*dt],-T_gripperlog) 
% % % % figure;plot([0:dt:(size(ControlSwitchlog,1)-1)*dt],ControlSwitchlog(:,2)) 
% % % % figure;plot([0:dt:(size(smoothing_factorlog,1)-1)*dt],smoothing_factorlog) 
% % % 
% % % % save("IROS_short.mat",'poscmdlog',"posfbklog","Telog","Felog","deflectionlog","Tc_poslog","Fc_poslog","Tc_forcelog","Fc_forcelog","Tmlog","ControlSwitchlog","Xelog","Velog","Xdlog","Vdlog",'T_gripperlog','smoothing_factorlog','xyzTargets','rotMatTarget','control_time','gripperforce','Fmlog')
% % % % load forcecontrol.mat


