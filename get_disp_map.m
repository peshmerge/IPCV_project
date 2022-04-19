function [dmap, unrel] = get_disp_map(subject,left,right,mask)

mask=rgb2gray(mask);
left=rgb2gray(left);
right=rgb2gray(right);

switch subject
case 'sub1' % Subject 1
    disparityRange = [276,340]; uThreshold = 5; dThreshold = 15;
case 'sub2' % Subject 2
    disparityRange = [292 372]; uThreshold = 1; dThreshold = 5;
case 'sub4' % Subject 4
    disparityRange = [256 512]; uThreshold = 5; dThreshold = 10;
end

dmap = disparity(left,right,'DisparityRange',disparityRange, ...
        'UniquenessThreshold',uThreshold,'DistanceThreshold',dThreshold);
dmap(imcomplement(mask > 0)) = 0; % Mask background
% Smoothing and Extend image at boundaries
dmap = medfilt2(dmap, [85 85],'symmetric');  

dmap = imfill(dmap,'holes'); %Remove holes
figure;imshow(dmap,disparityRange);
colormap(gca,jet);colorbar; %Depth colors

% Unreliable map
unrel = ones(size(dmap));
unrel(find(dmap~=0)) = 0;
unrel(find(dmap==-realmax('single'))) = 1; %Remove unreliable disparity

end
