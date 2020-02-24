function vision_Callback(~,message)
    disp('new pose is received');
    global vision_xyzTargets;
    global vision_rotMatTarget; 
    global flag;
    
    fprintf("Recived Position is %.3f %.3f %.3f\n",message.Position.X, message.Position.Y, message.Position.Z);
    fprintf("Recived Orientiation is %.3f %.3f %.3f %.3f\n",message.Orientation.X, message.Orientation.Y, message.Orientation.Z, message.Orientation.W);
    fprintf("flag is on\n")
    flag = 1;
    
%     vision_rotMatTarget = R_x(pi)*R_y(-pi/6);
    vision_xyzTargets = [message.Position.X message.Position.Y message.Position.Z]';
    vision_rotMatTarget = quat2rotm([message.Orientation.X message.Orientation.Y message.Orientation.Z message.Orientation.W]);
%     position = [message.Position.X message.Position.Y message.Position.Z];
%     orientation_quaternion = [message.Orientation.X message.Orientation.Y message.Orientation.Z message.Orientation.W];
%     HEBI_Demo5_BushingTestbed_vision2
end
