function [face] = k_means(image)
%% color based segmentation using k means clustering
lab = rgb2lab(image); % Convert image to L*a*b* color space
ab=lab(:,:,2:3);  % Extract tha a*b* values
r=size(ab,1);c=size(ab,2);
% create vectors of only a* and b* values
ab2=reshape(ab,r*c,2); 
nColors=3;             
% Repeat clustering n times to avoid local minima
[index,center]= kmeans(ab2,nColors,'Replicates',3); 

%% Label Pixel in the Image
% converting cluster classes into original image size
pixel_labels=reshape(index,r,c);
imshow(pixel_labels,[])
e=cell(1,nColors);
% replicate class output into 3 channels (rgb)
rgb= repmat(pixel_labels,[1 1 3]);
%% clustering each class
% set each channel to 0 if it does not match cluster class
for cluster=1:nColors
    img=image;
    img(rgb~=cluster)=0;

    % e contains cluster class for each class per 
    % RGB element of the original image
    e{cluster}=img;
    % figure;imshow(e{cluster})
end

%% Segment the face into a Separate Image
m=mean(center,2); % Sort the clusters
sub=image;
[~,index]=sort(m);
lvalue=lab(:,:,1); % Extract the L* values
face_index=find(pixel_labels==index(3)); %Face cluster
face_lvalue=lvalue(face_index); %get L* values of face cluster pixels
face_light_thres=imbinarize(rescale(face_lvalue));
face_labels=repmat(uint8(0),[r c]);
face_labels(face_index(face_light_thres==false))=1;
face_labels_rep=repmat(face_labels,[1 1 3]);
sub(face_labels_rep~=1)=0; 
mask=sub|e{3}; % Make binary mask 
mask2d=mask(:,:,1); 

% Perform morphological operations to isolate the face
E=imerode(mask2d,strel('disk',14));
maskrc=imreconstruct(E,mask2d);
maskmorphop=imopen(maskrc,strel('disk',12));
maskmorphcl=imclose(maskmorphop,strel('disk',50));
E1=imerode(maskmorphcl,strel('disk',24));
maskrc1= imreconstruct(E1,maskmorphcl);
face=maskrc1;
figure;imshow(face);

end