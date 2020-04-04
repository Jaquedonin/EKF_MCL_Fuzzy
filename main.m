%%
rosshutdown
clear velPub
clear velSub
clear odomSub
clear msg
ipaddress = '192.168.0.19';
rosinit(ipaddress);

gazebo = ExampleHelperGazeboCommunicator;
map = ExampleHelperGazeboModel('jersey_barrier','gazeboDB');
spawnModel(gazebo,map,[3.03 6.398 0],[0, 0, 0]);

[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
odomSub = rossubscriber('/odom', 'BufferSize', 25);
velSub = rossubscriber('/mobile_base/commands/velocity', 'BufferSize', 1);
laserSub = rossubscriber('/scan');
msg = rosmessage('geometry_msgs/Twist');

%%
positionRef = geraTraj('mapa.bmp');

Xref = positionRef(:,1);
Vx = diff(Xref);
Ax = diff(Vx);

Yref = positionRef(:,2);
Vy = diff(Yref);
Ay = diff(Vy);

fisObject = readfis("controlador_fuzzy2.fis");
fis = getFISCodeGenerationData(fisObject);
%%
image = imread('mapa.bmp');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
grid = robotics.BinaryOccupancyGrid(bwimage, 100);
figure
show(grid);
%%
for i = 1:length(Xref)
    
    msgOdom = receive(odomSub);
    hold on
    plot(msgOdom.Pose.Pose.Position.X, msgOdom.Pose.Pose.Position.Y, 'ro')
    ex = Xref(i) - msgOdom.Pose.Pose.Position.X;
    ey = Yref(i) - msgOdom.Pose.Pose.Position.Y;
    eTeta = Orientation(ex,ey) - RealOrientation(msgOdom.Pose.Pose.Orientation.Z,msgOdom.Pose.Pose.Orientation.W);
    disp(msgOdom.Pose.Pose.Position.X);
    disp(msgOdom.Pose.Pose.Position.Y);
    
    FuzzyOutput = fuzzy_mex(fis, [ex ey eTeta]);
    
    msg.Linear.X = FuzzyOutput(1);
    msg.Angular.Z = FuzzyOutput(2);
    send(velPub, msg); 
    
    pause(3.20);
end