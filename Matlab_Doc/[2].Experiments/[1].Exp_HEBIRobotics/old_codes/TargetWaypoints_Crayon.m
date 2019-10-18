function [xyzTargets, rotMatTarget, isgripper, gripperforce] = TargetWaypoints(demo_case)
    if demo_case == 11
            %%%%%%%%%%%%%%%%%%%%%% 크레파스 두개 패키징 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            %%%%% 위치 다시 세팅해야 함

        % xyzTargets = [ 0.47    0.47    0.47    0.47   0.47   0.47;    % x [m]
        %                0.145    0.145    0.145    0.093   0.093   0.093;    % y [m]
        %               -0.20   -0.37   -0.30   -0.30  -0.39  -0.20];  % z [m]

        xyzTargets = [ 0.55    0.55    0.55   0.455    0.455    0.455    0.455   0.455   0.455;    % x [m]
                       0.127   0.127   0.127   0.127    0.127    0.127    0.063   0.063   0.063;    % y [m]
                      -0.15   -0.28   -0.15   -0.15    -0.33    -0.20    -0.20   -0.33   -0.10];  % z [m]

        rotMatTarget{1} = R_x(pi);
        rotMatTarget{2} = R_x(pi); 
        rotMatTarget{3} = R_x(pi); 
        rotMatTarget{4} = R_x(pi); 
        rotMatTarget{5} = R_x(pi); 
        rotMatTarget{6} = R_x(pi); 
        rotMatTarget{7} = R_x(pi); 
        rotMatTarget{8} = R_x(pi); 
        rotMatTarget{9} = R_x(pi); 

        isgripper = [1 1 1 1 1 1 1 1];
        gripperforce = [-5 -5 -5 -5 -5 -5 -5 -5];

    elseif demo_case == 12
            %%%%%%%%%%%%%%%%%%% 크레파스를 옮기고 난 후 패키징 %%%%%%%%%%%%%%%%%%%%%%%
            %%%%% 위치 다시 세팅해야 함
        xyzTargets = [ 0.44   0.44   0.44   0.44   0.47   0.47   0.47    0.47    0.47    0.47;    % x [m]
                      -0.18  -0.18  -0.18  -0.18   0.145  0.145  0.145   0.145   0.145   0.145;  % y [m]
                      -0.15  -0.24  -0.24  -0.15  -0.20  -0.30  -0.30   -0.30   -0.37   -0.30];  % z [m]

        rotMatTarget{1} = R_x(pi);   
        rotMatTarget{2} = rotMatTarget{1};
        rotMatTarget{3} = rotMatTarget{2};     
        rotMatTarget{4} = rotMatTarget{3};
        rotMatTarget{5} = R_z(3*pi/2)*R_x(pi);
        rotMatTarget{6}= rotMatTarget{5};
        rotMatTarget{7}= rotMatTarget{6};
        rotMatTarget{8}=  R_x(pi);   
        rotMatTarget{9}=  rotMatTarget{8}; 
        rotMatTarget{10}=  rotMatTarget{9}; 

        %            2 3 4 5 6 7 8 9 10
        isgripper = [0 1 1 1 1 0 1 1 1];
        gripperforce = [1 -5 -5 -5 -5 -5 -5 -5 -5 -5];
    end
end

