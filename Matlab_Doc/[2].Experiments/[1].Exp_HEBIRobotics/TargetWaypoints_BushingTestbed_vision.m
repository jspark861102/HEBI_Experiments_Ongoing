function [posTargets, xyzTargets, rotMatTarget, control_time, gripperforce, FT_trigger, desired_force, num_init_move, IKinit] = TargetWaypoints_BushingTestbed(demo_case, kin, initPosition_front, initPosition_back)
    global vision_xyzTargets; 
    global vision_rotMatTarget; 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���׷κ��� �ν� pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    pick2.xyzTargets =   [ -0.20 ;    % x [m]
                           -0.43 ;    % y [m]
                            0.15 ];  % z [m]  
    pick2.gripperforce = [  1.5  ];  %���� ���۽ÿ� �ش� �� �Ϸ���    
    pick2.control_time = [  2.0  ]; 
        rot_2 = R_x(pi)*R_z(pi);
    pick2.rotMatTarget = [  rot_2];
    pick2.IKinit =       [ -1    ];
    pick2.FT_trigger =   [  0    ];
    pick2.desired_force =[  0    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���׷κ��� �ν� pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    pick3.xyzTargets =   [  0.285   0.285;    % x [m]
                           -0.265  -0.265;    % y [m]2
                           -0.080  -0.080   ];  % z [m]  
    pick3.gripperforce = [  -5       -5       ];
    pick3.control_time = [  2.0     10.0    ]; 
    pick3.rotMatTarget = [  R_x(pi) R_x(pi) ];
    pick3.IKinit =       [  1       1       ];
    pick3.FT_trigger =   [  0       0       ];
    pick3.desired_force =[  0       0       ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���׷κ��� �ν� pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
%     pick.xyzTargets =   [ 0.285   0.285   0.270;    % x [m]
%                          -0.265  -0.265  -0.255;    % y [m]
%                          -0.185  -0.185  -0.030];  % z [m]                      
%     pick.gripperforce = [ 2      -5      -5];  %���� ���۽ÿ� �ش� �� �Ϸ���    
%     pick.control_time = [ 1.0     0.8     1.0]; 
%     pick.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi)];    
%     pick.IKinit =       [ 1       1       1      ];
%     pick.FT_trigger =   [ 0       0       0      ];
%     pick.desired_force =[ 0       0       0      ];

    
    pick.xyzTargets =   [ [vision_xyzTargets(1:2);-0.080]     vision_xyzTargets      vision_xyzTargets     [0.270 -0.255 -0.030]']; 
    pick.gripperforce = [ 2                                   2                     -5                     -5                    ];    
    pick.control_time = [ 1.0                                 1.5                    0.8                    1.5                  ]; 
    pick.rotMatTarget = [ vision_rotMatTarget                 vision_rotMatTarget    vision_rotMatTarget    R_x(pi)              ];
    pick.IKinit =       [ 1                                   1                      1                      1                    ];
    pick.FT_trigger   = [ 0                                   0                      0                      0                    ];
    pick.desired_force= [ 0                                   0                      0                      0                    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% �ν� pushing 2 (���� ������ ����)%%%%%%%%%%%%%%%%%%%%
                              %1       2          
    pushing2.xyzTargets =   [ -0.4913  -0.4913  -0.4913;    % x [m]
                              -0.1000   0.2000  -0.0000;    % y [m]
                               0.1880   0.1880   0.3000];  % z [m]
    pushing2.gripperforce = [ -5       -2.0     -5    ];           
    pushing2.control_time = [  3.0        3        1    ];
        rot_2 = R_x(pi)*R_z(pi);
    pushing2.rotMatTarget = [  rot_2    rot_2    rot_2];
    pushing2.IKinit =       [ -1       -1       -1    ];
    pushing2.FT_trigger   = [  0        2        0    ];
    pushing2.desired_force= [  0       -11       0    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ���� 2%%%%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
    insert_end2.xyzTargets =    [ -0.4913  -0.4913  -0.4913;    % x [m]
                                   0.1300   0.1300  -0.0700;    % y [m]
                                   0.3000   0.1900   0.3500 ];  % z [m]   
    insert_end2.gripperforce =  [ -5       -5        3      ];  %���� ���۽ÿ� �ش� �� �Ϸ��� 
    insert_end2.control_time =  [  1.0      1.0      1.0      ]; 
        rot_2 = R_x(pi-pi/10)*R_z(pi);
    insert_end2.rotMatTarget =  [  rot_2    rot_2    rot_2    ];
    insert_end2.IKinit =        [ -1       -1       -1        ];
    insert_end2.FT_trigger =    [  0        0        0        ];
    insert_end2.desired_force =    [  0        0        0        ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
          
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ���� 2%%%%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
%     align.xyzTargets =   [ [vision_xyzTargets(1:2);-0.080]     [vision_xyzTargets(1:2);-0.080]      [vision_xyzTargets(1:2);-0.300]    [0.500 vision_xyzTargets(2) -0.300]'  [0.500 0.1 -0.300]'       [vision_xyzTargets(1:2);-0.080]]; 
%     align.gripperforce = [-5                                  -5                                   -5                                 -5                                    -5                        -1                              ];    
%     align.control_time = [ 1.0                                 0.8                                  2.0                                2.0                                   3.0                       3.0                            ]; 
%     align.rotMatTarget = [ vision_rotMatTarget                 vision_rotMatTarget                  vision_rotMatTarget                vision_rotMatTarget                   vision_rotMatTarget       vision_rotMatTarget            ];
%     align.IKinit =       [ 1                                   1                                    1                                  1                                     1                         1                              ];
%     align.FT_trigger   = [ 0                                   0                                    3                                  1                                     2                         0                              ];
%     align.desired_force= [ 0                                   0                                    30                                -16                                   -3                         0                              ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ���� 2%%%%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
%     align.xyzTargets =   [ [vision_xyzTargets(1:2);-0.080]     [vision_xyzTargets(1:2);-0.080]      [vision_xyzTargets(1:2);-0.300]    [0.500 vision_xyzTargets(2) -0.300]'  [0.500 vision_xyzTargets(2) -0.300]'    [0.500 0.1 -0.300]'       [vision_xyzTargets(1:2);-0.080]    [vision_xyzTargets(1:2);-0.080]]; 
%     align.gripperforce = [-5                                  -5                                   -5                                 -5                                    -5                                      -5                        -0.9                               -0.9];    
%     align.control_time = [ 1.0                                 0.8                                  2.0                                2.0                                   0.8                                     5.0                       0.5                                3.0]; 
%     align.rotMatTarget = [ vision_rotMatTarget                 vision_rotMatTarget                  vision_rotMatTarget                vision_rotMatTarget                   vision_rotMatTarget                     vision_rotMatTarget       vision_rotMatTarget                vision_rotMatTarget];
%     align.IKinit =       [ 1                                   1                                    1                                  1                                     1                                       1                         1                                  1];
%     align.FT_trigger   = [ 0                                   0                                    3                                  1                                     0                                       2                         0                                  0];
%     align.desired_force= [ 0                                   0                                    27                                -7                                     0                                      -0.1                       0                                  0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ���� 2%%%%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
    align.xyzTargets =   [ [vision_xyzTargets(1:2);-0.080]     [vision_xyzTargets(1:2);-0.080]      [vision_xyzTargets(1:2);-0.300]    [0.500 vision_xyzTargets(2) -0.300]'  [0.500 vision_xyzTargets(2) -0.300]'    [0.500 0.1 -0.300]'       [vision_xyzTargets(1:2);-0.080]]; 
    align.gripperforce = [-5                                  -5                                   -5                                 -5                                    -5                                      -5                        -1.1];    
    align.control_time = [ 1.0                                 0.8                                  2.0                                2.0                                   0.8                                     5.0                       3.0]; 
    align.rotMatTarget = [ vision_rotMatTarget                 vision_rotMatTarget                  vision_rotMatTarget                vision_rotMatTarget                   vision_rotMatTarget                     vision_rotMatTarget       vision_rotMatTarget];
    align.IKinit =       [ 1                                   1                                    1                                  1                                     1                                       1                         1];
    align.FT_trigger   = [ 0                                   0                                    3                                  1                                     0                                       2                         0];
    align.desired_force= [ 0                                   0                                    27                                -7                                     0                                      -1.1                       0];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      
    
     
    if demo_case == 1
            %%%%%%%% Ư����Ż ���輼���� pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick2.xyzTargets    pick3.xyzTargets     pick.xyzTargets     pushing2.xyzTargets     insert_end2.xyzTargets];
        gripperforce =     [pick2.gripperforce  pick3.gripperforce   pick.gripperforce   pushing2.gripperforce   insert_end2.gripperforce];           
        control_time =     [pick2.control_time  pick3.control_time   pick.control_time   pushing2.control_time   insert_end2.control_time];        
        rotMatTarget_Set = [pick2.rotMatTarget  pick3.rotMatTarget   pick.rotMatTarget   pushing2.rotMatTarget   insert_end2.rotMatTarget]; 
        IKinit =           [pick2.IKinit        pick3.IKinit         pick.IKinit         pushing2.IKinit         insert_end2.IKinit];
        FT_trigger =       [pick2.FT_trigger    pick3.FT_trigger     pick.FT_trigger     pushing2.FT_trigger     insert_end2.FT_trigger];
        desired_force =    [pick2.desired_force pick3.desired_force  pick.desired_force  pushing2.desired_force  insert_end2.desired_force];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end    
        num_init_move = 2;
    
    elseif demo_case == 2
            %%%%%%%% Ư����Ż ���輼���� pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick3.xyzTargets     align.xyzTargets];
        gripperforce =     [pick3.gripperforce   align.gripperforce];           
        control_time =     [pick3.control_time   align.control_time];        
        rotMatTarget_Set = [pick3.rotMatTarget   align.rotMatTarget]; 
        IKinit =           [pick3.IKinit         align.IKinit];
        FT_trigger =       [pick3.FT_trigger     align.FT_trigger];
        desired_force =    [pick3.desired_force  align.desired_force];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end        
        num_init_move = 1;
    end    
    
    for i=1:length(xyzTargets(1,:))
        if IKinit(i) == 1
            posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                         'SO3', rotMatTarget{i}, ...
                                         'initial', initPosition_front ); 
        elseif IKinit(i) == -1
            posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                         'SO3', rotMatTarget{i}, ...
                                         'initial', initPosition_back );
        end            
    end    
end

