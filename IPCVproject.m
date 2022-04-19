clear variables
close all
clc

%% Reading image
% Read all images from input dataset
% subject=imageSet('calibration\subject1', 'recursive');
% subject=imageSet('calibration\subject2', 'recursive');
subject=imageSet('calibration\subject4', 'recursive');

% load only required images
img_L = im2double(imread(subject(1).ImageLocation{3}));
img_M = im2double(imread(subject(2).ImageLocation{3}));
img_R = im2double(imread(subject(3).ImageLocation{3}));

% Stereo Camera Calibration
% Processed using Stereo camera calibrator application
% params calibrated using the calib app
load mid_left_params -mat
load mid_right_params -mat
stereoParamsC1ML=mid_left_params;
stereoParamsC1MR=mid_right_params;

figure('Name','Stero params C1 LM');showExtrinsics(stereoParamsC1ML);
title('Stero params ML camera pair'); print -r150 -dpng camera1-ML-params.png

figure('Name','Stero params C1 MR');showExtrinsics(stereoParamsC1MR);
title('Stero params MR camera pair'); print -r150 -dpng camera1-MR-params.png

%% Face extraction from images using k-means clustering
% left image
[mask_L] = get_face_mask(img_L);
figure('Name','Mask cut L image');imshow(mask_L,[]);title('Face cut from image L');
% imwrite(mask_L,'faceS1L.jpg')

%% middle image
[mask_M] = get_face_mask(img_M);
figure('Name','Mask cut M image');imshow(mask_M,[]);title('Face cut from image M');
% imwrite(mask_M,'faceS1M.jpg')

%% right image
[mask_R] = get_face_mask(img_R);
figure('Name','Mask cut R image');imshow(mask_R,[]);title('Face cut from image R')
% imwrite(mask_R,'faceS1R.jpg')

%% save masked face images to disk
% save maskS1 mask_L mask_M mask_R 
% save maskS2 mask_L mask_M mask_R 
% save maskS4 mask_L mask_M mask_R 

%% Load masked faces from disk
% load maskS1.mat
% load maskS2.mat
% load maskS4.mat

%% Stereo rectification
% Rectify the images of the subject
% Taking M -> L
[img_ML_rec, img_L_rec]=rectifyStereoImages(img_M, img_L,...
    stereoParamsC1ML,'OutputView','full');
figure('Name','Rectified image pair ML');imshowpair(img_L_rec,img_ML_rec);
title('Stereo rectified image pair ML');
% print -r150 -dpng recfullMLS1.png

% taking M -> R
[img_MR_rec,img_R_rec]=rectifyStereoImages(img_M,img_R, ...
    stereoParamsC1MR, 'OutputView','full');
figure('Name','Rectified image pair MR');imshowpair(img_MR_rec,img_R_rec);
title('Stereo rectified image pair MR');
% print -r150 -dpng recfullMRS1.png

% Rectify the masked faces of the subject
[img_ML_recm,img_L_recm]=rectifyStereoImages(mask_M, mask_L, ...
    stereoParamsC1ML,'OutputView','full');
figure('Name','Rectified face pair ML');imshowpair(img_L_recm,img_ML_recm);
title('Stereo rectified face pair ML');
% print -r150 -dpng recfaceMLS1.png

[img_MR_recm,img_R_recm]=rectifyStereoImages(mask_M,mask_R, ...
    stereoParamsC1MR, 'OutputView','full');
figure('Name','Rectified face pair MR');imshowpair(img_MR_recm,img_R_recm);
title('Stereo rectified image pair MR');
% print -r150 -dpng recfaceMRS1.png

%% Disparity map
% Processed using dispmap function created, subjective to input images,
% thus the first parameter
[dmap_LM,unrel_LM]=get_disp_map('sub4',img_L_rec,img_ML_rec,img_L_recm);
title("Disparity map ML");
% print -r150 -dpng dispMLS1.png

[dmap_MR,unrel_MR]=get_disp_map('sub4',img_MR_rec,img_R_rec,img_MR_recm);
title("Disparity map MR");
% print -r150 -dpng dispMRS1.png

%% 3D point clouds
% Create point clouds
scene3d_ML = reconstructScene(dmap_LM,stereoParamsC1ML);

%% Attempt at merging the point clouds for single mesh
pc_ML = pointCloud(scene3d_ML);pc_MR = pointCloud(scene3d_MR);
dn_ML= pcdenoise(pc_ML);dn_MR = pcdenoise(pc_MR);
ds_ML = pcdownsample(dn_ML,'nonuniformGridSample',6);
ds_MR = pcdownsample(dn_MR,'nonuniformGridSample',6);

figure('Name', 'Point cloud ML');pcshow(ds_ML)
title('Point cloud ML')
% print -r150 -dpng pointcloudMLS1.png

scene3d_MR = reconstructScene(dmap_MR,stereoParamsC1MR);
figure('Name', 'Point cloud MR');pcshow(ds_MR)
title('Point cloud MR');
% print -r150 -dpng pointcloudMRS1.png

% [tform,mreg,rmse]= pcregistericp(ds_ML, ds_MR,'Extrapolate',true);
% ptcloudout=pcmerge(ds_MR,mreg,1);
%% Create 3D face meshe from point clouds
% Copied function from provided handouts
mesh_LM = mesh_3D(dmap_LM,scene3d_ML,unrel_LM,img_L_recm);
% Adjust according to output
xlim([-100 150]);ylim([-200 200]);zlim([-750 -400]);
title('ML Mesh');
% print -r150 -dpng facemeshMLS1.png

mesh_MR = mesh_3D(dmap_MR,scene3d_MR,unrel_MR,img_MR_recm);
% Adjust according to output
xlim([100 350]);ylim([-200 200]);zlim([300 700]);
title('MR Mesh'); 
% print -r150 -dpng facemeshMRS1.png;