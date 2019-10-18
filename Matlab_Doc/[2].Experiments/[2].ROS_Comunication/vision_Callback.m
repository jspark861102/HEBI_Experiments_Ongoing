function vision_Callback(~,message)
    disp('new pose is received');
    global position 
    global orientation_quaternion 
    global flag
    
    flag = 1;
    position = [message.Position.X message.Position.Y message.Position.Z];
    orientation_quaternion = [message.Orientation.X message.Orientation.Y message.Orientation.Z message.Orientation.W];
    HEBI_Demo5_BushingTestbed_vision2
end