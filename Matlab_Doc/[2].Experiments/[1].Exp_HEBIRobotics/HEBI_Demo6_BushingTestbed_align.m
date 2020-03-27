%%
% clear *;
% close all;
% clc

%% setting parmaeters
%2:align
demo_case = 2;

%controlmode = 1 : cartesian space control
%controlmode = 2 : joint pace control
controlmode = 1;

%gravity setting 1 : z axis
%gravity setting 2 : axis based on base gyro sensor
gravitysetting = 1;

% force control smooting
smoothing_duration = 0.2; % sec

%% object parameters
object_case = 1; 

%% HEBI setting
% HebiLookup.initialize();
% [kin,gains,trajGen,group,cmd,grippergroup,grippercmd] = HEBI_Arm_Initialize;

% group.startLog('dir','logs');
           
%% Target Waypoints    
% Inverse Kinematics initial position
initPosition_front  = [  0   pi/4 pi/2 pi/4 -pi   pi/2 ];  % [rad]
initPosition_back   = [ -pi  pi/4 pi/2 pi/4 -pi   pi/2 ];  % [rad]
initPosition_front2 = [  0   pi/4 pi/2 pi/4 -pi   pi ];  % [rad]
initPosition_back2  = [ -pi  pi/4 pi/2 pi/4 -pi/2 pi ];  % [rad]
[posTargets, xyzTargets, rotMatTarget, control_time, gripperforce, FT_trigger, desired_force, num_init_move, IKinit] = TargetWaypoints_BushingTestbed_vision(object_case, demo_case, kin);

%% gravity direction
[gravityVec] = HEBI_Arm_gravity(gravitysetting);

%% log data setting
poscmdlog = [];posfbklog = [];
Telog = [];Felog = [];deflectionlog = [];Tc_poslog = [];Fc_poslog = [];Tc_forcelog = [];Fc_forcelog = [];Tmlog = [];
ControlSwitchlog = []; Xelog = []; Velog = []; Xdlog = []; Vdlog = [];T_gripperlog = [];smoothing_factorlog = [];Fmlog = [];
%% trajectory & control
real_dt_set=[];
for iter=1:1

%%%%%%%%%%%%%%%%%%%%% go from here to first waypoint %%%%%%%%%%%%%%%%%%%%
%control setting
fbk = group.getNextFeedbackFull(); 
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
    
    grippercmd.position = [];
    grippercmd.velocity = [];
    grippercmd.effort = gripperforce(1);    
    grippergroup.send(grippercmd);  
end    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% wait for vision %%%%%%%%%%%%%%%%%%%%%%%%%%%%
for i=1:num_init_move  
    waypoints = [ posTargets(i,:) ;
                  posTargets(i+1,:) ];
    trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)] );
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

        grippercmd.position = [];
        grippercmd.velocity = [];
        grippercmd.effort = gripperforce(i+1);    
        grippergroup.send(grippercmd);  
    end
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%% go next waypoints %%%%%%%%%%%%%%%%%%%%%%%%%%%%
%initial condition
ControlSwitch = [0 0 0 0 0 0];
Xe = [xyzTargets(:,1);pi;0;0]'; 
Xd = [xyzTargets(:,1);pi;0;0]';
% align_xyzTargets = vision_xyzTargets; 
align_xyzTargets = xyzTargets; 
for i=num_init_move+1:size(posTargets,1)-1   
    
%     if any(ControlSwitch) 
%         waypoints = [ fbk.position ;
%                       posTargets(i+1,:) ];
%     else        
%         waypoints = [ posTargets(i,:) ;
%                       posTargets(i+1,:) ];
%     end  
%     
%     trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%% modified code for align %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if i == num_init_move+1+2 %force contact through z axis
        align_xyzTargets(:,i+1) = [xyzTargets(1,i+1); xyzTargets(2,i+1); -0.3];
    elseif i == num_init_move+1+2+1 %force contact through x axis
        align_xyzTargets(:,i+1) = [0.5; Xe(1,2); Xe(1,3)+0.01];
    elseif i ==  num_init_move+1+2+2 %move back to -x axis
        align_xyzTargets(:,i+1) = [Xe(1,1) - 0.015; Xe(1,2); Xe(1,3)];
    elseif i ==  num_init_move+1+2+3 %force contact through y axis
        align_xyzTargets(:,i+1) = [Xe(1,1) - 0.01; 0.1; Xe(1,3)];
    elseif  i ==  num_init_move+1+2+4 %pause not to disturb bushing
        align_xyzTargets(:,i+1) = [Xe(1,1)+0.01; Xe(1,2)+0.00; Xe(1,3)];
    elseif  i ==  num_init_move+1+2+5 %go up for bushing length with pushing
        align_xyzTargets(:,i+1) = [Xe(1,1)+0.03; Xe(1,2)+0.03; Xe(1,3)+0.06];
    elseif  i ==  num_init_move+1+2+6 %go up to air
        align_xyzTargets(:,i+1) = [Xe(1,1); Xe(1,2); -0.08];
    end       
    if IKinit(i+1) == 1
        newposTarget = kin.getIK( 'xyz', align_xyzTargets(:,i+1), ...
                                  'SO3', rotMatTarget{i+1}, ...
                                  'initial', initPosition_front );
    elseif IKinit(i+1) == -1
        newposTarget = kin.getIK( 'xyz', align_xyzTargets(:,i+1), ...
                                  'SO3', rotMatTarget{i+1}, ...
                                  'initial', initPosition_back );
                              
    elseif IKinit(i+1) == 2
        newposTarget = kin.getIK( 'xyz', align_xyzTargets(:,i+1), ...
                                  'SO3', rotMatTarget{i+1}, ...
                                  'initial', initPosition_front2 );
    elseif IKinit(i+1) == -2
        newposTarget = kin.getIK( 'xyz', align_xyzTargets(:,i+1), ...
                                  'SO3', rotMatTarget{i+1}, ...
                                  'initial', initPosition_back2 );
    end
    if i > num_init_move+1+2 
        waypoints = [ fbk.position ;
                      newposTarget ];        
    else
        waypoints = [ posTargets(i,:) ;
                      posTargets(i+1,:)];
    end
    if i > num_init_move+1+2+4  
%         trajectory = trajGen.newLinearMove( waypoints, 'duration', control_time(i+1));
        trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    else
        trajectory = trajGen.newJointMove( waypoints, 'time', [0 control_time(i+1)]);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    t0 = fbk.time;
    t = 0; pre_t = 0; 
    desired_force_array = zeros(1,6); 
    ControlSwitch = [0 0 0 0 0 0];   
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
        dt = 1/group.getFeedbackFrequency;
        Ve = (J * fbk.velocity')';
%         Xe = Xe + Ve * dt;
        Xe = Xe + Ve * real_dt;
        
        %get external torque exerted on joint and endeffector
        Te = fbk.deflection.*[70 170 70 70 70 70];
        Fe = (inv(J')*Te')';       
        
        if FT_trigger(i+1) ~= 0
            i_switch = FT_trigger(i+1);
            desired_force_array(i_switch) = desired_force(i+1);
            if ControlSwitch(i_switch) == 0 
                if desired_force(i+1) > 0 
                    if Fe(i_switch) >= desired_force(i+1)                     
                       ControlSwitch(i_switch) = 1;
                       is_newwaypoint = 1;
                       fprintf("force control position is %d \n",i_switch);
                       desired_force_array
                       ControlSwitch
                       
                       %%%%%% additional code for align %%%%%%
                       dumy = real_dt_set; 
                       real_dt_set = [];
                       real_dt_set = dumy(:,1:end-1);
                       fprintf("force control time is %.4f \n",t);
                       fprintf("%d th waypoint \n",i);
                       disp('%%%%%%%%%%%%%%%%%%%%%%%%%')
                       break;
                       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%                       
                    end
                elseif desired_force(i+1) < 0 
                    if Fe(i_switch) <= desired_force(i+1)
                       ControlSwitch(i_switch) = 1;
                       is_newwaypoint = 1;
                       fprintf("force control position is %d \n",i_switch);
                       desired_force_array  
                       ControlSwitch
                       
                       %%%%%% additional code for align %%%%%%
                       dumy = real_dt_set; 
                       real_dt_set = [];
                       real_dt_set = dumy(:,1:end-1);
                       fprintf("force control time is %.4f \n",t);
                       fprintf("%d th waypoint \n",i);
                       disp('%%%%%%%%%%%%%%%%%%%%%%%%%')
                       break;
                       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                    end
                end                        
            end
            if trajectory.getDuration - t <= smoothing_duration
                smoothing_factor = 1/smoothing_duration * (trajectory.getDuration - t);
%                 smoothing_factor = 1;   
            end            
        end
        
        if is_newwaypoint == 1                                    
            newwaypoint = [ fbk.position ;
                          fbk.position ];
            trajectory = trajGen.newJointMove( newwaypoint, 'time', [0 control_time(i+1)]);
            is_newwaypoint = 0;
        end    
        [pos,vel,acc] = trajectory.getState(t);
                
        % Account for external efforts due to the gas spring
        effortOffset = [0 -7.5+2.26*(fbk.position(2) - 0.72) 0 0 0 0];

        gravCompEfforts = kin.getGravCompEfforts( fbk.position, gravityVec );
        dynamicCompEfforts = kin.getDynamicCompEfforts( fbk.position, ...
                                                        pos, vel, acc);   
        Tm = dynamicCompEfforts + gravCompEfforts + effortOffset;   
        
        %for align, force control gain is modified, smoothing_factor is negelected
        Fc_force = (([0.5 0.5 0.5 0 0 0].*(Fe - desired_force_array.*[1 1 1 1 1 1])) - Fe).*ControlSwitch; 
        Tc_force = (J' * Fc_force')';% - gains.positionKp.*(fbk.position - pos)*any(ControlSwitch);
               
        if any(ControlSwitch)       
            Xd = Xd .* ([1 1 1 1 1 1] - ControlSwitch) + Xe .* ControlSwitch; 
            Vd = Vd .* ([1 1 1 1 1 1] - ControlSwitch) + Ve .* ControlSwitch; 
        else 
            Jd = kin.getJacobian('endeffector',pos); 
            Vd = (Jd * vel')';
%             Xd = Xd + Vd * dt;
            Xd = Xd + Vd * real_dt;

        end
        
        %for align, position gain is modified
        ePgain = [20 20 5 9 9 6];
%         ePgain = [100 100 150 9 9 6];
%         ePgain = [80 80 130 20 20 16];
%         ePgain = [100 100 5 9 9 6];        
        eVgain = [0.1 0.1 0.1 0.1 0.1 0.1]; 
        Fc_pos = -ePgain .* (Xe - (Xd + [0 0 0 0 0 0])) - eVgain .* (Ve - Vd); 
        Fc_pos = Fc_pos .* ([1 1 1 1 1 1] - ControlSwitch); 
        Tc_pos = (J' * Fc_pos')'*any(ControlSwitch);      
%         Tc_pos = (J' * Fc_pos')'*any(FT_trigger(i+1));        

        
        cmd.position = pos;
        cmd.velocity = [];
        cmd.effort = Tm + Tc_force + Tc_pos;        
        group.send(cmd);   
        
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
end
%% plot
% Stop logging and plot the command vs feedback pos/vel/effort
% log = group.stopLog();

% HebiUtils.plotLogs( log, 'position');
% HebiUtils.plotLogs( log, 'velocity');
% HebiUtils.plotLogs( log, 'effort');

% subplot6a([0:dt:(size(Telog,1)-1)*dt],Telog,'Te',control_time)  
t_set = cumsum(real_dt_set);
% subplot6a(t_set,Felog,'Fe',control_time)  

figure;plot(t_set,Felog(:,1))  
grid on
title('contact force')
xlabel('time')
ylabel('force(N)')
set(gcf, 'Position', [10 40 1500 700])

figure;plot(t_set,Felog(:,2))  
grid on
title('contact force')
xlabel('time')
ylabel('force(N)')
set(gcf, 'Position', [10 40 1500 700])

figure;plot(t_set,Felog(:,3))  
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


