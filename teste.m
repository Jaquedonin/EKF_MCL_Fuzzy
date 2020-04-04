[obstacles, xf, yf, current] = gertraj();
%%
rosshutdown
clear velPub
clear velSub
clear odomSub
clear msg
ipaddress = '192.168.0.144';
rosinit(ipaddress);
% 
gazebo = ExampleHelperGazeboCommunicator;
map = ExampleHelperGazeboModel('jersey_barrier','gazeboDB');
spawnModel(gazebo,map,[3.03 6.398 0],[0, 0, 0]);

[velPub, velMsg] = rospublisher('/mobile_base/commands/velocity', 'geometry_msgs/Twist');
odomSub = rossubscriber('/odom', 'BufferSize', 25);
velSub = rossubscriber('/mobile_base/commands/velocity', 'BufferSize', 1);
laserSub = rossubscriber('/scan');
msg = rosmessage('geometry_msgs/Twist');

current{1} = [0.0 0.0];
xref = repelem(0.6,length(current) - 2);
yref = repelem(0.0,length(current) - 2);
xref(1) = 0.0;

for v = 2:length(current)-1
   xref(v-1) = current{v}(1) - 0.6;
   yref(v-1) = current{v}(2) - 0.0;
end 

Vx = diff(xref);
aX = diff(Vx);

Vy = diff(yref);
aY = diff(Vy);

Xrefp = 0;
Yrefp = 0;
TRst = 0;
TRsv = 0;
TRsw = 0;
Xref = 0;
Yref = 0;
PHIref = 0;
Vref = 0;
Wref = 0;
TRsx = 0;
TRsy = 0;

figure
plot(xref, yref,'go');
hold on;
%%
%Inicializa o modelo de odometria

image = imread('mapateste.bmp');
grayimage = rgb2gray(image);
bwimage = grayimage < 0.5;
grid = robotics.BinaryOccupancyGrid(bwimage, 100);

odometryModel = robotics.OdometryMotionModel;
odometryModel.Noise = [0.2 0.2 0.2 0.2];

rangeFinderModel = robotics.LikelihoodFieldSensorModel;
rangeFinderModel.SensorLimits = [0.45 8];
rangeFinderModel.Map = grid;

%% Transformação ROS
tftree = rostf;
waitForTransform(tftree,'/base_link','/camera_depth_frame');
sensorTransform = getTransform(tftree,'/base_link','/camera_depth_frame');

% Get the euler rotation angles.
laserQuat = [sensorTransform.Transform.Rotation.W,sensorTransform.Transform.Rotation.X, sensorTransform.Transform.Rotation.Y,sensorTransform.Transform.Rotation.Z];
laserRotation = quat2eul(laserQuat, 'ZYX');

%% Configura rangeFinder
rangeFinderModel.SensorPose = [sensorTransform.Transform.Translation.X , sensorTransform.Transform.Translation.Y ,laserRotation(1)];

%% Inicia objeto do AMCL
amcl = robotics.MonteCarloLocalization;
amcl.UseLidarScan = true;
amcl.MotionModel = odometryModel;
amcl.SensorModel = rangeFinderModel;
amcl.UpdateThresholds = [0.2,0.2,0.2];
amcl.ResamplingInterval = 1;


amcl.ParticleLimits = [5000 10000];
amcl.GlobalLocalization = false;
amcl.InitialPose = ExampleHelperAMCLGazeboTruePose;
%amcl.InitialPose = [0 0 0];
amcl.InitialCovariance = eye(3)*0.5;

visualizationHelper = ExampleHelperAMCLVisualization(grid);

wanderHelper = ExampleHelperAMCLWanderer(laserSub, sensorTransform, velPub, velMsg);

%% Inicia EKF
EKF = trackingEKF(@constvel,@cvmeas,[0;0;0;0], ...
    'StateTransitionJacobianFcn',@constveljac, ...
    'MeasurementJacobianFcn',@cvmeasjac);

%% Loop de Controle
for i = 1:length(current)-4
    
   Xrefp = xref(i);
   Yrefp = yref(i);
   
   [Vout_MPC, Wout_MPC] = MPC(Xrefp, Yrefp, TRsx, TRsy, TRst, TRsv, TRsw,Xref, Yref, PHIref, Vref, Wref);
   
   msg.Linear.X = Vout_MPC;
   msg.Angular.Z = Wout_MPC;
   send(velPub, msg);
   
   pause(1.5);
   
   msgOdom = receive(odomSub);
   
   TRsx = msgOdom.Pose.Pose.Position.X;
   TRsy = msgOdom.Pose.Pose.Position.Y;
   TRst = calcTeta(msgOdom.Pose.Pose.Orientation.Z,msgOdom.Pose.Pose.Orientation.W);
   
   disp('MEDIDA');
   measurement = [TRsx; TRsy; TRst];
   %Ler o scan
   disp(measurement);
   scanMsg = receive(laserSub);
   
   scan = lidarScan(scanMsg);
   
   [xcorr, Pcorr] = correct(EKF,measurement');
   
   disp('xcor');
   disp(xcorr);
   
   teta = atan2(xcorr(2),xcorr(4));
   pose = [xcorr(1), xcorr(3), teta];
   
   [isUpdated, estimatedPose, estimatedCovarience] = amcl(measurement,scan);
   
   TRsv = velSub.LatestMessage.Linear.X;
   TRsw = velSub.LatestMessage.Angular.Z;
   
   [PHIref, Vref, Wref] = calcTetaVW(Vx(i), aX(i), Vy(i), aY(i));
   Xref = xref(i);
   Yref = yref(i);
   
   if(isUpdated) 
        disp(estimatedPose)
        plotStep(visualizationHelper, amcl, estimatedPose, scan, i);
        TRsx = estimatedPose(1);  
        TRsy = estimatedPose(2);
        TRst = estimatedPose(3);
        plot(TRsx + 0.6,TRsy,'ro');
   else
       hold on;
       plot(TRsx + 0.6,TRsy,'ro');
   
   end
   
   disp(i);
   
end 

