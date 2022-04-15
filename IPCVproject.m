clear variables
close all
clc

%% Reading image
% Read all images from input dataset
subject1=imageSet('calibration\subject1', 'recursive');
subject2=imageSet('calibration\subject2', 'recursive');
subject4=imageSet('calibration\subject4', 'recursive');
%% Select subject
% load only required images
% sub=subject1;
% sub=subject2;
sub=subject4;
img_L = im2double(imread(sub(1).ImageLocation{3}));
img_M = im2double(imread(sub(2).ImageLocation{3}));
img_R = im2double(imread(sub(3).ImageLocation{3}));

%% Stereo Camera Calibration
%% Processed using Stereo camera calibrator application
% params calibrated using the calib app
load mid_left_params -mat
load mid_right_params -mat
stereoParamsC1ML=mid_left_params;
stereoParamsC1MR=mid_right_params;

% figure('Name','Stero params C1 LM');showExtrinsics(stereoParamsC1ML);
% title('Stero params ML camera pair'); print -r150 -dpng camera1-ML-params.png
% figure('Name','Stero params C1 MR');showExtrinsics(stereoParamsC1MR);
% title('Stero params MR camera pair'); print -r150 -dpng camera1-MR-params.png

%% Mask of the subject using k-means clustering
%% Processed using kmeans function created
% left
[mask_L] = k_means(img_L);
mask_L1=cat(3,mask_L,mask_L,mask_L);
mask_L2 = img_L;
mask_L2(imcomplement(mask_L1))=0;
figure('Name','Mask L');imshow(mask_L,[]);title('Face mask image L S4');
figure('Name','Mask cut L image');imshow(mask_L2,[]);title('Face cut from image L S4');
%% save mask and face cut
% imwrite(mask_L,'maskS4L.jpg')
% imwrite(mask_L2,'faceS4L.jpg')

%%
[mask_M] = k_means(img_M);
mask_M1=cat(3,mask_M,mask_M,mask_M);
mask_M2 = img_M;
mask_M2(imcomplement(mask_M1))=0;
figure('Name','Mask M');imshow(mask_M,[]);title('Face mask image M S4');
figure('Name','Mask cut M image');imshow(mask_M2,[]);title('Face cut from image M S4');
%% save mask and face cut
% imwrite(mask_M,'maskS4M.jpg')
% imwrite(mask_M2,'faceS4M.jpg')

%%
[mask_R] = k_means(img_R);
mask_R1=cat(3,mask_R,mask_R,mask_R);
mask_R2 = img_R;
mask_R2(imcomplement(mask_R1))=0;
figure('Name','Mask R');imshow(mask_R,[]);title('Face mask image R S4')
figure('Name','Mask cut R image');imshow(mask_R2,[]);title('Face cut from image R S4')
%% save mask and face cut
% imwrite(mask_R,'maskS4R.jpg')
% imwrite(mask_R2,'faceS4R.jpg')

%%
% save maskS1 mask_L mask_L2 mask_M mask_M2 mask_R mask_R2
% save maskS2 mask_L mask_L2 mask_M mask_M2 mask_R mask_R2
% save maskS4 mask_L mask_L2 mask_M mask_M2 mask_R mask_R2

%% Once mask extracted, saved and used for future executions
% load maskS1.mat
% load maskS2.mat
% load maskS4.mat

%% Stereo rectification
% Rectify the images of the subject
% Taking M -> L
[img_ML_rec, img_L_rec]=rectifyStereoImages(img_M, img_L,...
    stereoParamsC1ML,'OutputView','full');
figure('Name','Rectified image pair ML');imshowpair(img_L_rec,img_ML_rec);
title('Stereo rectified image pair ML S4');
% print -r150 -dpng recfullMLS4.png
    
%%
% taking M -> R
[img_MR_rec,img_R_rec]=rectifyStereoImages(img_M,img_R, ...
    stereoParamsC1MR, 'OutputView','full');
figure('Name','Rectified image pair MR');imshowpair(img_MR_rec,img_R_rec);
title('Stereo rectified image pair MR S4');
% print -r150 -dpng recfullMRS4.png

%%
% Rectify the masked images of the subject
[img_ML_recm,img_L_recm]=rectifyStereoImages(mask_M2, mask_L2, ...
    stereoParamsC1ML,'OutputView','full');
figure('Name','Rectified face pair ML');imshowpair(img_L_recm,img_ML_recm);
title('Stereo rectified face pair ML S4');
% print -r150 -dpng recfaceMLS4.png
        
%%
[img_MR_recm,img_R_recm]=rectifyStereoImages(mask_M2,mask_R2, ...
    stereoParamsC1MR, 'OutputView','full');
figure('Name','Rectified face pair MR');imshowpair(img_MR_recm,img_R_recm);
title('Stereo rectified image pair MR S4');
% print -r150 -dpng recfaceMRS4.png

%% Stereo matching
% Processed by Registration Estimator application
% [stmatch_LM]= SMLM4(img_L_recm, img_LM_recm);
% print -r150 -dpng SMS4LM.png
% [stmatch_MR]= SMMR4(img_MR_recm, img_R_recm);
% print -r150 -dpng SMS4MR.png

%% Disparity map
%% Processed using dispmap function created
[dmap_LM,unrel_LM]=dispmap('S4',img_L_rec,img_ML_rec,img_L_recm);
title("Disparity map ML S4");
% print -r150 -dpng dispMLS4.png
%%
[dmap_MR,unrel_MR]=dispmap('S4',img_MR_rec,img_R_rec,img_MR_recm);
title("Disparity map MR S4");
% print -r150 -dpng dispMRS4.png

%% 3D point clouds
% Create point clouds
scene3d_ML = reconstructScene(dmap_LM,stereoParamsC1ML);
scene3d_MR = reconstructScene(dmap_MR,stereoParamsC1MR);

%% Attempt at merging the point clouds for single mesh
pc_ML = pointCloud(scene3d_ML);pc_MR = pointCloud(scene3d_MR);
dn_ML= pcdenoise(pc_ML);dn_MR = pcdenoise(pc_MR);
ds_ML = pcdownsample(dn_ML,'nonuniformGridSample',6);
ds_MR = pcdownsample(dn_MR,'nonuniformGridSample',6);

% [tform,mreg,rmse]= pcregistericp(ds_MR, ds_ML,'Extrapolate',true);
% ptcloudout=pcmerge(ds_ML,mreg,1);
%%
figure('Name', 'Downsampled point cloud ML S4');pcshow(ds_ML)
title('Downsampled point cloud ML S4')
% print -r150 -dpng pointcloudMLS4.png
%%
figure('Name', 'Downsampled point cloud MR S4');pcshow(ds_MR)
title('Downsampled point cloud MR S4');
% print -r150 -dpng pointcloudMRS4.png

%% Create 3D meshes from point clouds
% Processed using function from UT syllabi
mesh_LM = mesh_3D(dmap_LM,scene3d_ML,unrel_LM,img_L_recm);
% Adjust according to output
% xlim([-100 150]);ylim([-200 200]);zlim([-750 -400]);
title('ML Mesh S4');
% print -r150 -dpng facemeshMLS4.png
%%
mesh_MR = mesh_3D(dmap_MR,scene3d_MR,unrel_MR,img_MR_recm);
% Adjust according to output
% xlim([100 350]);ylim([-200 200]);zlim([300 700]);
title('MR Mesh S4'); 
% print -r150 -dpng facemeshMRS4.png;