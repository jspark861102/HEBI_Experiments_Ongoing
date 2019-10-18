clc
close all
clear all

%% initialization
masterHost = 'localhost';
Matlab_node = robotics.ros.Node('Matlab_node', masterHost);
request_pub = robotics.ros.Publisher(Matlab_node,'/request', 'std_msgs/Int32');
posemsg_Sub = robotics.ros.Subscriber(Matlab_node,'/pose','geometry_msgs/Pose', @vision_Callback);

%% variables
global position 
global orientation_quaternion
global flag

flag = 0;
position = [0 0 0];
orientation_quaternion = [0 0 0 0];


disp('initial pose is');
position
orientation_quaternion

%% move
for i = 1 : 3
    %% move to tray


    %% request to vision to obtain /pose
    requestmsg = rosmessage(request_pub);
    requestmsg.Data = 1;
    disp('request is published')
    send(request_pub,requestmsg) %send하면 /pose 값이 돌아고고, vision_callback이 실행됨.
    pause(2)
    if flag ==1
        %global position, orientation 값 받음    
        disp('obtained pose is');
        position
        orientation_quaternion
        flag = 0
    else
        disp('pose is not obtained')
    end
    

    %% move and pushing
end

