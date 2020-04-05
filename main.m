%%
rosshutdown
clear velPub
clear velSub
clear odomSub
clear msg
ipaddress = '192.168.0.11';
rosinit(ipaddress);

gazebo = ExampleHelperGazeboCommunicator;
map = ExampleHelperGazeboModel('jersey_barrier','gazeboDB');
spawnModel(gazebo,map,[10.527347 9.670895 0],[0, 0, 0]);

[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
odomSub = rossubscriber('/odom', 'BufferSize', 25);
velSub = rossubscriber('/mobile_base/commands/velocity', 'BufferSize', 1);
laserSub = rossubscriber('/scan');
msg = rosmessage('geometry_msgs/Twist');

%%
positionRef = geraTraj('mapa.bmp');

%InitialX = [1.17;];
Xref = positionRef(:,1);
%Xref = [InitialX;Xref];
Vx = diff(Xref);
Ax = diff(Vx);


%InitialY = [-0.35;];
Yref = positionRef(:,2);
%Yref = [InitialY;Yref];
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
EKF = trackingEKF(@constvel,@cvmeas,[0;0;0;0], ...
    'StateTransitionJacobianFcn',@constveljac, ...
    'MeasurementJacobianFcn',@cvmeasjac);
measurement = [0;0;0];
[xpred, Ppred] = predict(EKF,3);
[xcorr, Pcorr] = correct(EKF,measurement);
[xpred, Ppred] = predict(EKF,3);
%%
odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = grid;

% Query the Transformation Tree (tf tree) in ROS.
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link','/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W,sensorTransform.Transform.Rotation.X, sensorTransform.Transform.Rotation.Y,sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');


% Setup the |SensorPose|, which includes the translation along base_link's
% +X, +Y direction in meters and rotation angle alongbase_link's +Z axis in radians.

rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X , sensorTransform.Transform.Translation.Y ,laserRotation(1)];

laserSub = rossubscriber('/scan');
odomSub = rossubscriber('/odom');
[velPub,velMsg] = rospublisher('/mobile_base/commands/velocity','geometry_msgs/Twist');

amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;


amcl.ParticleLimits = [500 5000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(grid);

wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%%
for i = 1:length(Xref)
    msgOdom = receive(odomSub);
    
    Xreal = msgOdom.Pose.Pose.Position.X;
    Yreal = msgOdom.Pose.Pose.Position.Y;
    TetaReal = RealOrientation(msgOdom.Pose.Pose.Orientation.Z,msgOdom.Pose.Pose.Orientation.W);
    
    measurement = [Xreal;Yreal;TetaReal];
    
    [xcorr, Pcorr] = correct(EKF, measurement);
    lastBestGuess = xcorr(1:3);
    
    lastBestGuess = [lastBestGuess(1);lastBestGuess(3);Orientation(lastBestGuess(1),lastBestGuess(3))];
    
    scanMsg = receive(laserSub);
    scan = lidarScan(scanMsg);
    
    [isUpdated, estimatedPose, estimatedCovarience] = amcl(measurement,scan);
    
    if(isUpdated) 
        disp(estimatedPose)
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i);
        Xreal = estimatedPose(1);
        Yreal = estimatedPose(2);
        TetaReal = estimatedPose(3); 
        plot(Xref(i),Yref(i),'x');
        plot(Xreal,Yreal,'ro');
    else
       hold on;
       plot(Xref(i),Yref(i),'x');
       plot(Xreal,Yreal,'ro');
    end
    
    %hold on
    %plot(msgOdom.Pose.Pose.Position.X, msgOdom.Pose.Pose.Position.Y, 'ro') 
    ex = Xref(i) - Xreal;
    ey = Yref(i) - Yreal;
    eTeta = Orientation(ex,ey) - TetaReal;
    
    FuzzyOutput = fuzzy_mex(fis, [ex ey eTeta]);
    
    msg.Linear.X = FuzzyOutput(1);
    msg.Angular.Z = FuzzyOutput(2);
    send(velPub, msg); 
     
    pause(2);
end