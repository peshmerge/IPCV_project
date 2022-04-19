function [face] = get_face_mask(image)
%% color based k means clustering
% Using L*a*b* color space - to extract only the colors
lab = rgb2lab(image);
ab=lab(:,:,2:3); 
r=size(ab,1);c=size(ab,2);
% create vectors of a* and b* values for k-means algo
ab_vec=reshape(ab,r*c,2); 
no_of_colors=3;
[index,~]= kmeans(ab_vec,no_of_colors,'Replicates',3); 

% label original image pixels according to their classes
pixel_labels=reshape(index,r,c);
e=cell(1,no_of_colors);
% replicate class output into 3 rgb channels
rgb_labels = repmat(pixel_labels,[1 1 3]);

% masking per class
% set each channel to 0 if it does not match cluster class
for cluster=1:no_of_colors
    img=image;
    img(rgb_labels~=cluster)=0;

    % e contains cluster class for each class per 
    % RGB element of the original image
    e{cluster}=img;
    % figure;imshow(e{cluster})
end

%% Isolate face as a mask
% RANDOMLY considering 3rd cluster as face cluster
mask2d=imbinarize(e{3}(:,:,1));

% Perform morphological operations to isolate the face
% Parameters are based on trial and error to get good face isolation
element=imerode(mask2d,strel('disk',14));
mask_rc=imreconstruct(element,mask2d);
mask_open=imopen(mask_rc,strel('disk',12));
mask_close=imclose(mask_open,strel('disk',50));

% replicate to 3 channels and mask original image to get only face
mask_rgb_channels=cat(3,mask_close,mask_close,mask_close);
face = image;
face(imcomplement(mask_rgb_channels))=0;

end