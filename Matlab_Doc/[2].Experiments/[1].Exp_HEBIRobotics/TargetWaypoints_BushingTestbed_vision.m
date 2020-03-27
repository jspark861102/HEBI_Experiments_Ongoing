function [posTargets, xyzTargets, rotMatTarget, control_time, gripperforce, FT_trigger, desired_force, num_init_move, IKinit] = TargetWaypoints_BushingTestbed_vision(object_case, demo_case, kin)
    %% explanation
     %neutral : far left position to be a bridge of + and - initPosition problem
     %toLbox : go to Lbox
     %toRbox : go to Rbox
     %pick_vision : pick with the loacation obtained from vision
     %pushing_bed : push the bed with bushing with hybrid position/force control
     %insert_bed  : insert bushing, sequential step with pushing_bed
     %pickup : pick and tilting object
     %insert_bed2 : insert bushing as human do, sequential step with pickup
     %align_vision : align bushing into Lbox based on vision

    % Inverse Kinematics initial position
    initPosition_front  = [  0   pi/4 pi/2 pi/4 -pi   pi/2 ];  % [rad]
    initPosition_back   = [ -pi  pi/4 pi/2 pi/4 -pi   pi/2 ];  % [rad]
    initPosition_front2 = [  0   pi/4 pi/2 pi/4 -pi   pi ];  % [rad]
    initPosition_back2  = [ -pi  pi/4 pi/2 pi/4 -pi/2 pi ];  % [rad]

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% neutral %%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    neutral.xyzTargets =   [ -0.20 ;    % x [m]
                           -0.43 ;    % y [m]
                            0.15 ];  % z [m]  
    neutral.gripperforce = [  1.5  ];    
    neutral.control_time = [  2.0  ]; 
        rot_2 = R_x(pi)*R_z(pi);
    neutral.rotMatTarget = [  rot_2];
    neutral.IKinit =       [ -1    ];
    neutral.FT_trigger =   [  0    ];
    neutral.desired_force =[  0    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% toLbox %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    toLbox.xyzTargets =   [  0.285;    % x [m]
                            -0.265;    % y [m]2
                            -0.080  ];  % z [m]  
    toLbox.gripperforce = [ -5      ];
    toLbox.control_time = [  4.0    ]; 
    toLbox.rotMatTarget = [  R_x(pi)];
    toLbox.IKinit =       [  1      ];
    toLbox.FT_trigger =   [  0      ];
    toLbox.desired_force =[  0      ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%     
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%% toRbox %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
                         %1       2       3       4   
    toRbox.xyzTargets =   [  0.285;    % x [m]
                            +0.265;    % y [m]2
                            -0.080   ];  % z [m]  
    toRbox.gripperforce = [ -0.5     ];
    toRbox.control_time = [  5.0     ]; 
    toRbox.rotMatTarget = [  R_x(pi) ];
    toRbox.IKinit =       [  1       ];
    toRbox.FT_trigger =   [  0       ];
    toRbox.desired_force =[  0       ];

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%     global vision_xyzTargets; 
%     global vision_rotMatTarget; 
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pick_vision %%%%%%%%%%%%%%%%%%%%%%%%%%
%                          %1                                   2                      3                     4       
%     pick_vision.xyzTargets =   [ [vision_xyzTargets(1:2);-0.080]     vision_xyzTargets      vision_xyzTargets     [0.270 -0.255 -0.030]']; 
%     pick_vision.gripperforce = [ 2                                   2                     -5                     -5                    ];    
%     pick_vision.control_time = [ 1.0                                 1.5                    0.8                    1.5                  ]; 
%     pick_vision.rotMatTarget = [ vision_rotMatTarget                 vision_rotMatTarget    vision_rotMatTarget    R_x(pi)              ];
%     pick_vision.IKinit =       [ 1                                   1                      1                      1                    ];
%     pick_vision.FT_trigger   = [ 0                                   0                      0                      0                    ];
%     pick_vision.desired_force= [ 0                                   0                      0                      0                    ];
%     %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
   %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% pushing_bed %%%%%%%%%%%%%%%%%%%%%%
                              %1       2          
    pushing_bed.xyzTargets =   [ -0.4913  -0.4913  -0.4913;    % x [m]
                              -0.1000   0.2000  -0.0000;    % y [m]
                               0.1960   0.1960   0.3000];  % z [m]
    pushing_bed.gripperforce = [ -5       -1.3     -5    ];           
    pushing_bed.control_time = [  3.0        3        1    ];
        rot_pushing = R_x(pi)*R_z(pi);
    pushing_bed.rotMatTarget = [  rot_pushing    rot_pushing    rot_pushing];
    pushing_bed.IKinit =       [ -1       -1       -1    ];
    pushing_bed.FT_trigger   = [  0        2        0    ];
    pushing_bed.desired_force= [  0       -11       0    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
      
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% insert_bed %%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
    insert_bed.xyzTargets =    [ -0.4913  -0.4913  -0.4913;    % x [m]
                                   0.1300   0.1300  -0.0500;    % y [m]
                                   0.3000   0.1900   0.3500 ];  % z [m]   
    insert_bed.gripperforce =  [ -5       -5        3      ];  
    insert_bed.control_time =  [  1.0      1.0      1.0      ]; 
        rot_insert = R_x(pi-pi/10)*R_z(pi);
    insert_bedpickup.rotMatTarget =  [  rot_insert    rot_insert    rot_insert    ];
    insert_bed.IKinit =        [ -1       -1       -1        ];
    insert_bed.FT_trigger =    [  0        0        0        ];
    insert_bed.desired_force =    [  0        0        0        ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% insert_bed2 %%%%%%%%%%%%%%%%%%%%%
                                  %1       2          
    insert_bed2.xyzTargets =    [ -0.4976          -0.4000                 -0.4000                  -0.4000               ;    % x [m]
                                  -0.3052           0.0800                  0.0900                  -0.0800               ;    % y [m]
                                   0.3000           0.2800                  0.2000                   0.2000               ];  % z [m]   
    insert_bed2.gripperforce =  [ -5               -5                      -5                       -0.0                    ];  
    insert_bed2.control_time =  [  5.0              5.0                     5.0                      5.0                  ]; 
    insert_bed2.rotMatTarget =  [  R_x(pi)*R_z(pi)  R_y(-pi/2)*R_x(-pi/2)   R_y(-pi/2)*R_x(-pi/2)    R_y(-pi/2)*R_x(-pi/2)];
    insert_bed2.IKinit =        [ -1               -2                      -2                       -2                    ];
    insert_bed2.FT_trigger =    [  0                0                       0                        0                    ];
    insert_bed2.desired_force = [  0                0                       0                        0                    ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
     
    align_xyzTargets = [0.315 -0.180 -0.185]'; %initial value
    align_rotMatTarget =  R_x(pi)*R_z(pi*3/4);
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% align_vision %%%%%%%%%%%%%%%%%%%
    align_vision.xyzTargets =   [ [align_xyzTargets(1:2);-0.080]     [align_xyzTargets(1:2);-0.080]      [align_xyzTargets(1:2);-0.300]    [0.600 align_xyzTargets(2) -0.300]'  [0.600 align_xyzTargets(2) -0.300]'    [0.600 0.1 -0.300]'        [align_xyzTargets(1:2);-0.300]     [align_xyzTargets(1:2);-0.080]     [align_xyzTargets(1:2);-0.080]]; 
    align_vision.gripperforce = [-5                                 -5                                  -5                                -5                                   -5                                     -5                         -3                                  0.20                               0.20                          ];    
    align_vision.control_time = [ 1.0                                0.8                                 2.0                               2.0                                  0.8                                    5.0                        1.0                                2.0                                2.0                           ]; 
    align_vision.rotMatTarget = [ align_rotMatTarget                 align_rotMatTarget                  align_rotMatTarget                align_rotMatTarget                   align_rotMatTarget                     align_rotMatTarget         align_rotMatTarget                 align_rotMatTarget                 align_rotMatTarget            ];
    align_vision.IKinit =       [ 2                                  2                                   2                                 2                                    2                                      2                          2                                  2                                  2                             ];
    align_vision.FT_trigger   = [ 0                                  0                                   3                                 1                                    0                                      2                          0                                  0                                  0                             ];
    align_vision.desired_force= [ 0                                  0                                   32                               -8                                    0                                     -1.8                        0                                  0                                  0                             ];
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
      
    global l;
    global D;  
    global vision_xyzTargets; 
    global vision_rotMatTarget; 
    xi = vision_xyzTargets(1)-0.058;
    yi = vision_xyzTargets(2)+0.018;
%     zi = vision_xyzTargets(3);
    rot_vision = vision_rotMatTarget;
    if object_case == 1    
        %bushing 1
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.3;
        zl = (l-D)*0.5;
        
%         xi = 0.285;
%         yi = 0.270;
%         zi = -0.266;
        rot_pickup = R_x(pi)*R_y(pi/10);
        
%         xi = 0.285;
%         yi = 0.270;
        zi = -0.266;
%         rot_pickup = rot_vision;

        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4)+0.02)     xi+xl*(1-cos(pi/2/5*4)+0.23)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi                        yi                        yi                       yi                               yi+0.03                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)-0.02         zi+zl*sin(pi/2/5*1)+0.04        -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -3            -3                        -3                        -3                       -3                               -2.0                             -5.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                              1.0                              2.0                         ];
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*3.0/10)          R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                                1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                                0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                                0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    elseif object_case == 5
        %cookie can
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.9;
        zl = (l-D)*0.8;
        xi = 0.225;
        yi = 0.270;
        zi = -0.234;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4))     xi+xl*(1-cos(pi/2/5*4)+0.07)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi-0.02                   yi-0.02                   yi-0.03                  yi+0.03                     yi+0.03                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)+0.01    zi+zl*sin(pi/2/5*1)+0.06        -0.0                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -5            -5                        -5                        -5                       -5                          -0.5                             -5.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                         2.0                              2.0                         ];
            rot_pickup = R_x(pi)*R_y(pi/3.8);
%             rot_pickup = R_x(pi)*R_y(pi/10);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*2.9/10)     R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                           1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                           0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                           0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    elseif object_case == 6
        %water bottle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.9;
        zl = (l-D)*0.8;
        xi = 0.225;
        yi = 0.270;
        zi = -0.238;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4))     xi+xl*(1-cos(pi/2/5*4)+0.07)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi                        yi                        yi                       yi                          yi+0.03                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)         zi+zl*sin(pi/2/5*1)+0.04        -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -5            -5                        -5                        -5                       -5                          -2.0                             -5.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                         1.0                              2.0                         ];
%             rot_pickup = R_x(pi)*R_y(pi/6);
            rot_pickup = R_x(pi)*R_y(pi/4);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*2.8/10)     R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                           1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                           0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                           0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    elseif object_case == 9
        %medicine bottle
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.5;
        zl = (l-D)*0.5;
        xi = 0.285;
        yi = 0.270;
        zi = -0.254;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4))     xi+xl*(1-cos(pi/2/5*4)+0.07)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi                        yi                        yi                       yi                          yi+0.04                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)-0.02    zi+zl*sin(pi/2/5*1)+0.04        -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -3            -3                        -3                        -3                       -3                          -2.0                             -3.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                         2.0                              2.0                         ];
            rot_pickup = R_x(pi)*R_y(pi/10);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*3.7/10)     R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                           1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                           0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                           0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    elseif object_case == 10
        %plastic cup
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.5;
        zl = (l-D)*0.5;
        xi = 0.285;
        yi = 0.270;
        zi = -0.250;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4))     xi+xl*(1-cos(pi/2/5*4)+0.07)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi                        yi                        yi                       yi                          yi+0.04                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)-0.02    zi+zl*sin(pi/2/5*1)+0.06        -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -3            -3                        -3                        -3                       -3                          -2.0                             -3.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                         2.0                              2.0                         ];
            rot_pickup = R_x(pi)*R_y(pi/10);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*2.5/10)     R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                           1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                           0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                           0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    elseif object_case == 11
        %portruded thin object (mounting rail)
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.5;
        zl = (l-D)*0.5;
        xi = 0.285;
        yi = 0.270;
        zi = -0.275;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4)+0.06)     xi+xl*(1-cos(pi/2/5*4)+0.00)     xi+xl*(1-cos(pi/2/5*4)-0.0) ;    % x [m]
                                 yi           yi           yi            yi                        yi                        yi-0.03                  yi-0.03                          yi-0.03                          yi                          ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)-0.02         zi+zl*sin(pi/2/5*1)+0.09        -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -3            -3                        -3                        -3                       -3                               -1.0                             -3.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                              3.0                              2.0                         ];
            rot_pickup = R_x(pi)*R_y(pi/20);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2/10)    R_x(pi)*R_y(-pi*4.0/10)          R_x(pi)                          R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                                1                                1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                                0                                0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                                0                                0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    elseif object_case == 12
        %wiring duct
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%% pickup %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        xl = l*1.3;
        zl = (l-D)*0.9;
        xi = 0.285;
        yi = 0.270;
        zi = -0.235;
        pickup.xyzTargets =   [  xi           xi           xi            xi+xl*(1-cos(pi/2/5*1))   xi+xl*(1-cos(pi/2/5*2))   xi+xl*(1-cos(pi/2/5*3))  xi+xl*(1-cos(pi/2/5*4))     xi+xl*(1-cos(pi/2/5*4)+0.40)     xi+xl*(1-cos(pi/2/5*4)+0.40)    xi+xl*(1-cos(pi/2/5*4)+0.40)      xi+xl*(1-cos(pi/2/5*4)+0.40);    % x [m]
                                 yi           yi           yi            yi-0.00                   yi-0.02                   yi-0.02                  yi-0.02                     yi+0.02                          yi+0.02                         yi+0.02                           yi+0.02                     ;    % y [m]
                                 zi           zi           zi            zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)       zi+zl*sin(pi/2/5*1)      zi+zl*sin(pi/2/5*1)-0.02    zi+zl*sin(pi/2/5*1)+0.11         zi+zl*sin(pi/2/5*1)+0.02        zi+zl*sin(pi/2/5*1)+0.02         -0.1                         ];   % z [m]  
        pickup.gripperforce = [  1.5          0.5         -3            -3                        -3                        -3                       -3                          +0.0                             -0.0                            -3.0                              -3.0                         ];
        pickup.control_time = [  2.0          0.5          2.0           2.0                       2.0                       2.0                      2.0                         3.0                              2.0                             2.0                               2.0                         ];
            rot_pickup = R_x(pi)*R_y(pi/10);
        pickup.rotMatTarget = [  rot_pickup   rot_pickup   rot_pickup    R_x(pi)*R_y(0)            R_x(pi)*R_y(-pi*1/10)     R_x(pi)*R_y(-pi*2.5/10)  R_x(pi)*R_y(-pi*3.8/10)     R_x(pi)                          R_x(pi)                         R_x(pi)                           R_x(pi)                     ];
        pickup.IKinit =       [  1            1            1             1                         1                         1                        1                           1                                1                               1                                 1                           ];
        pickup.FT_trigger =   [  0            0            0             0                         0                         0                        0                           0                                0                               0                                 0                           ];
        pickup.desired_force =[  0            0            0             0                         0                         0                        0                           0                                0                               0                                 0                           ];
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    end

    
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if demo_case == 1 %pick and place w/ hybrid position/force control based pushing, pick_vision is used conceptually
%         xyzTargets =       [neutral.xyzTargets    toLbox.xyzTargets     pick_vision.xyzTargets     pushing_bed.xyzTargets     insert_bed.xyzTargets];
%         gripperforce =     [neutral.gripperforce  toLbox.gripperforce   pick_vision.gripperforce   pushing_bed.gripperforce   insert_bed.gripperforce];           
%         control_time =     [neutral.control_time  toLbox.control_time   pick_vision.control_time   pushing_bed.control_time   insert_bed.control_time];        
%         rotMatTarget_Set = [neutral.rotMatTarget  toLbox.rotMatTarget   pick_vision.rotMatTarget   pushing_bed.rotMatTarget   insert_bed.rotMatTarget]; 
%         IKinit =           [neutral.IKinit        toLbox.IKinit         pick_vision.IKinit         pushing_bed.IKinit         insert_bed.IKinit];
%         FT_trigger =       [neutral.FT_trigger    toLbox.FT_trigger     pick_vision.FT_trigger     pushing_bed.FT_trigger     insert_bed.FT_trigger];
%         desired_force =    [neutral.desired_force toLbox.desired_force  pick_vision.desired_force  pushing_bed.desired_force  insert_bed.desired_force];
%         for i = 1 : size(xyzTargets,2)          
%             rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
%         end    
%         num_init_move = 2; % the move step that interacts with vision
        
    elseif demo_case == 11 %insert bushing with sequence of pickup
        xyzTargets =       [insert_bed2.xyzTargets     toRbox.xyzTargets];
        gripperforce =     [insert_bed2.gripperforce   toRbox.gripperforce ];           
        control_time =     [insert_bed2.control_time   toRbox.control_time];        
        rotMatTarget_Set = [insert_bed2.rotMatTarget   toRbox.rotMatTarget]; 
        IKinit =           [insert_bed2.IKinit         toRbox.IKinit];
        FT_trigger =       [insert_bed2.FT_trigger     toRbox.FT_trigger];
        desired_force =    [insert_bed2.desired_force  toRbox.desired_force];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end    
        num_init_move = 0; %no vision interaction
    
    elseif demo_case == 2 %aligning bushing, distinctive running file is needed.
        xyzTargets =       [toLbox.xyzTargets     align_vision.xyzTargets];
        gripperforce =     [toLbox.gripperforce   align_vision.gripperforce];           
        control_time =     [toLbox.control_time   align_vision.control_time];        
        rotMatTarget_Set = [toLbox.rotMatTarget   align_vision.rotMatTarget]; 
        IKinit =           [toLbox.IKinit         align_vision.IKinit];
        FT_trigger =       [toLbox.FT_trigger     align_vision.FT_trigger];
        desired_force =    [toLbox.desired_force  align_vision.desired_force];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end        
        num_init_move = 0; %no vision interaction
        
    elseif demo_case == 3 %pick and tilt the bushing w/o vision
        xyzTargets =       [toRbox.xyzTargets     pickup.xyzTargets   ];
        gripperforce =     [toRbox.gripperforce   pickup.gripperforce ];           
        control_time =     [toRbox.control_time   pickup.control_time ];        
        rotMatTarget_Set = [toRbox.rotMatTarget   pickup.rotMatTarget ]; 
        IKinit =           [toRbox.IKinit         pickup.IKinit       ];
        FT_trigger =       [toRbox.FT_trigger     pickup.FT_trigger   ];
        desired_force =    [toRbox.desired_force  pickup.desired_force];
        for i = 1 : size(xyzTargets,2)          
            rotMatTarget{i} = rotMatTarget_Set(:,3*i-2:3*i);          
        end        
        num_init_move = 0;    
        
    elseif demo_case == 33 %pick and tilt the bushing w/ vision
        xyzTargets =       [toRbox.xyzTargets     toRbox.xyzTargets     pickup.xyzTargets   ];
        gripperforce =     [toRbox.gripperforce   toRbox.gripperforce   pickup.gripperforce ];           
        control_time =     [toRbox.control_time   toRbox.control_time   pickup.control_time ];        
        rotMatTarget_Set = [toRbox.rotMatTarget   toRbox.rotMatTarget   pickup.rotMatTarget ]; 
        IKinit =           [toRbox.IKinit         toRbox.IKinit         pickup.IKinit       ];
        FT_trigger =       [toRbox.FT_trigger     toRbox.FT_trigger     pickup.FT_trigger   ];
        desired_force =    [toRbox.desired_force  toRbox.desired_force  pickup.desired_force];
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
        elseif IKinit(i) == 2
            posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                         'SO3', rotMatTarget{i}, ...
                                         'initial', initPosition_front2 );
                                     
         elseif IKinit(i) == -2
            posTargets(i,:) = kin.getIK( 'xyz', xyzTargets(:,i), ...
                                         'SO3', rotMatTarget{i}, ...
                                         'initial', initPosition_back2 );
        end            
    end    
end

