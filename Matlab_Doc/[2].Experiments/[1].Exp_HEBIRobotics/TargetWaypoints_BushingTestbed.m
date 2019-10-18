function [posTargets, xyzTargets, rotMatTarget, control_time, gripperforce, FT_trigger] = TargetWaypoints_BushingTestbed(demo_case, kin, initPosition_front, initPosition_back)

    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���׷κ��� �ν� pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    pick.xyzTargets =   [ 0.285   0.285   0.285   0.270;    % x [m]
                         -0.265  -0.265  -0.265  -0.255;    % y [m]
                         -0.080  -0.185  -0.185  -0.030];  % z [m]  
    pick.gripperforce = [-5       2      -5      -5];  %���� ���۽ÿ� �ش� �� �Ϸ���    
    pick.control_time = [ 2.0     1.0     0.8     1.0]; 
    pick.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi) R_x(pi)];
    pick.FT_trigger =   [ 0       0       0       0      ];
    pick.IKinit =       [ 1       1       1       1      ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% ���׷κ��� �ν� pick %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    pick2.xyzTargets =   [ -0.20 ;    % x [m]
                           -0.43 ;    % y [m]
                            0.15 ];  % z [m]  
    pick2.gripperforce = [  1.5  ];  %���� ���۽ÿ� �ش� �� �Ϸ���    
    pick2.control_time = [  1.5  ]; 
        rot_2 = R_x(pi)*R_z(pi);
    pick2.rotMatTarget = [  rot_2];
    pick2.FT_trigger =   [  0    ];
    pick2.IKinit =       [ -1    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
      
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ����%%%%%%%%%%%%%%%%%%%%%%%
                                   %1       2       3      4       
    insert_end.xyzTargets =    [ 0.470   0.470   0.470   0.470  0.470;    % x [m]
                                 0.040   0.105   0.094   0.094  0.094;    % y [m]
                                 0.100   0.100  -0.025  -0.025  0.070];  % z [m]   
    insert_end.gripperforce =  [-5      -5      -5       3      3    ];  %���� ���۽ÿ� �ش� �� �Ϸ��� 
    insert_end.control_time =  [ 1.5     1.5     1.5     0.5    1    ]; 
        rot_i = R_x(pi-pi/4);
    insert_end.rotMatTarget = [ R_x(pi)  R_x(pi) rot_i   rot_i  rot_i];
    insert_end.FT_trigger =   [ 0        0       0       0      0    ];
    insert_end.IKinit =       [ 1       1       1       1       1      ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% �ν� pushing %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                            %1       2       3       4  
    pushing.xyzTargets =   [ 0.510   0.510   0.510;    % x [m]
                            -0.100   0.200  -0.050;    % y [m]
                            -0.025  -0.025  -0.025];  % z [m]
    pushing.gripperforce = [-5      -1.3    -5 ]; %-1.3           
    pushing.control_time = [ 1.5     3.0     1.0 ];
    pushing.rotMatTarget = [ R_x(pi) R_x(pi) R_x(pi)];
    pushing.FT_trigger   = [ 0       2       0      ]; %2�� Y���� ������ �ϰڴٴ� ��
    pushing.IKinit =       [ 1       1       1      ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% �ν� pushing 2 (���� ������ ����)%%%%%%%%%%%%%%%%%%%%
%                            %1       2          
    pushing2.xyzTargets =   [ -0.4913  -0.4913  -0.4913;    % x [m]
                              -0.1000   0.2000  -0.0000;    % y [m]
                               0.1860   0.1860   0.3000];  % z [m]
    pushing2.gripperforce = [ -5       -1.1     -5    ];           
    pushing2.control_time = [  3.0        3        1    ];
        rot_2 = R_x(pi)*R_z(pi);
    pushing2.rotMatTarget = [  rot_2    rot_2    rot_2];
    pushing2.FT_trigger   = [  0        2        0    ];
    pushing2.IKinit =       [ -1       -1       -1    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%% �׸��۸� ���Ա� ���� �� �������� �ν� ���� 2%%%%%%%%%%%%%%%%%%%%%%%
%                           %1       2          
    insert_end2.xyzTargets =    [ -0.4913  -0.4913  -0.4913;    % x [m]
                                   0.1370   0.1370  -0.0700;    % y [m]
                                   0.3000   0.1900   0.3500 ];  % z [m]   
    insert_end2.gripperforce =  [ -5       -5        3      ];  %���� ���۽ÿ� �ش� �� �Ϸ��� 
    insert_end2.control_time =  [  1.0      1.0      1.0      ]; 
        rot_2 = R_x(pi-pi/10)*R_z(pi);
    insert_end2.rotMatTarget = [   rot_2    rot_2    rot_2    ];
    insert_end2.FT_trigger =   [   0        0        0        ];
    insert_end2.IKinit =       [  -1       -1       -1        ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  
     
    if demo_case == 13
            %%%%%%%% Ư����Ż ���� pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick.xyzTargets     pushing.xyzTargets    insert_end.xyzTargets  ];
        gripperforce =     [pick.gripperforce   pushing.gripperforce  insert_end.gripperforce];           
        control_time =     [pick.control_time   pushing.control_time  insert_end.control_time];        
        rotMatTarget_Set = [pick.rotMatTarget   pushing.rotMatTarget  insert_end.rotMatTarget]; 
        FT_trigger =       [pick.FT_trigger     pushing.FT_trigger    insert_end.FT_trigger  ];
        IKinit =           [pick.IKinit         pushing.IKinit        insert_end.IKinit  ];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end         
        
    elseif demo_case == 14
            %%%%%%%% Ư����Ż ���輼���� pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick2.xyzTargets    pick.xyzTargets     pushing2.xyzTargets     insert_end2.xyzTargets];
        gripperforce =     [pick2.gripperforce  pick.gripperforce   pushing2.gripperforce   insert_end2.gripperforce];           
        control_time =     [pick2.control_time  pick.control_time   pushing2.control_time   insert_end2.control_time];        
        rotMatTarget_Set = [pick2.rotMatTarget  pick.rotMatTarget   pushing2.rotMatTarget   insert_end2.rotMatTarget]; 
        FT_trigger =       [pick2.FT_trigger    pick.FT_trigger     pushing2.FT_trigger     insert_end2.FT_trigger];
        IKinit =           [pick2.IKinit        pick.IKinit         pushing2.IKinit         insert_end2.IKinit];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end    
    
    elseif demo_case == 15
            %%%%%%%% Ư����Ż ���輼���� pick&pushing&insert %%%%%%%%%%
        xyzTargets =       [pick2.xyzTargets    pick.xyzTargets     pushing2.xyzTargets     insert_end2.xyzTargets  pick2.xyzTargets    pick.xyzTargets     pushing2.xyzTargets     insert_end2.xyzTargets  pick2.xyzTargets    pick.xyzTargets     pushing2.xyzTargets     insert_end2.xyzTargets];
        gripperforce =     [pick2.gripperforce  pick.gripperforce   pushing2.gripperforce   insert_end2.gripperforce  pick2.gripperforce  pick.gripperforce   pushing2.gripperforce   insert_end2.gripperforce  pick2.gripperforce  pick.gripperforce   pushing2.gripperforce   insert_end2.gripperforce  ];           
        control_time =     [pick2.control_time  pick.control_time   pushing2.control_time   insert_end2.control_time  pick2.control_time  pick.control_time   pushing2.control_time   insert_end2.control_time  pick2.control_time  pick.control_time   pushing2.control_time   insert_end2.control_time];        
        rotMatTarget_Set = [pick2.rotMatTarget  pick.rotMatTarget   pushing2.rotMatTarget   insert_end2.rotMatTarget  pick2.rotMatTarget  pick.rotMatTarget   pushing2.rotMatTarget   insert_end2.rotMatTarget  pick2.rotMatTarget  pick.rotMatTarget   pushing2.rotMatTarget   insert_end2.rotMatTarget]; 
        FT_trigger =       [pick2.FT_trigger    pick.FT_trigger     pushing2.FT_trigger     insert_end2.FT_trigger  pick2.FT_trigger    pick.FT_trigger     pushing2.FT_trigger     insert_end2.FT_trigger  pick2.FT_trigger    pick.FT_trigger     pushing2.FT_trigger     insert_end2.FT_trigger];
        IKinit =           [pick2.IKinit        pick.IKinit         pushing2.IKinit         insert_end2.IKinit  pick2.IKinit        pick.IKinit         pushing2.IKinit         insert_end2.IKinit  pick2.IKinit        pick.IKinit         pushing2.IKinit         insert_end2.IKinit];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end           
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

