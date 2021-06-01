% Obstacle Avoidance using Optical Flow
% Copyright 2021 The MathWorks, Inc.

% Model Initialization
% Start Location [X Z Y]
dronestartlocation=[-1.5 0 4];

% Final Location [X Z Y]
helipadLocation = [14 0.29 4];

% Linear Velocity [Vx Vz Vy]
linearVelocity = [0.6 0.05 .2]; 

% Angular Velocity during Optical Flow navigation
% represented as [pitch yaw roll]
angularVelocity = [0 0 -0.01]; 

% X-coordinate to switch from Optical flow controller to PID Controller
switchingXLimit = 12.0;

% Angular Velocity during landing
angularVelocityforLanding = [0 0 0.01];

% Hover Height before landing
finalHoverHeight = 0.8;

% Threshold distance to execute the PID controller.
delta = 0.1;



