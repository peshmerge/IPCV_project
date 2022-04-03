% 3D Face reconstruction 
% University Of Twente

%%
% Use Imtool to check the size of the checkerboard square size
% imtool('calibration\Calibratie 1\calibrationMiddle\Calibratie 1_M_1.jpg')

% Start Camera Calibrator App
% stereoCameraCalibrator;

%%
% Load the sessions for mid_left and mid_right into the calibrator app and
% save the sterorparams and the estimation errors 

% stereoCameraCalibrator('calibrationSession_mid_left.mat');
% save('stereoParams_mid_left.mat');
% save('estimationErrors_mid_left.mat');

% stereoCameraCalibrator('calibrationSession_mid_right.mat');
% save('stereoParams_mid_right.mat');
% save('estimationErrors_mid_right.mat');


% Load the stereo parameters and errors for mid_left and mid_right
load('stereoParams_mid_left.mat');
load('estimationErrors_mid_left.mat');
figure(1); 
showExtrinsics(stereoParams_mid_left);


% load('stereoParams_mid_right.mat');
% load('estimationErrors_mid_right.mat');
% figure(2); 
% showExtrinsics(stereoParams_mid_right);
