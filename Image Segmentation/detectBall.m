% Robotics: Estimation and Learning 
% WEEK 1
% 
% Complete this function following the instruction. 
function [segI, loc] = detectBall(I)
% function [segI, loc] = detectBall(I)
%
% INPUT
% I       120x160x3 numerial array 
%
% OUTPUT
% segI    120x160 numeric array
% loc     1x2 or 2x1 numeric array 



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Hard code your learned model parameters here
%
 muf = [147.4086  143.2787   62.9626];
 icovf = inv([  233.8396  145.9795 -236.5254
  145.9795  143.6211 -181.4336
 -236.5254 -181.4336  376.7108]);
detcov = 1/det(icovf);

 thre = 5e-5;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Find ball-color pixels using your model
% 
background(:,:,1) = muf(1)*ones(120,160);
background(:,:,2) = muf(2)*ones(120,160);
background(:,:,3) = muf(3)*ones(120,160);
I = double(I);
dI = I - background;

for i = 1:120
    for j = 1:160
        II = reshape(dI(i,j,:),1,3);
       pdis(i,j) =  exp(-0.5*II*icovf*II')/sqrt(2*pi*detcov);
    end
end
BW = reshape((pdis > thre),120,160);
%image(pdis);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Do more processing to segment out the right cluster of pixels.
% You may use the following functions.
%   bwconncomp
%   regionprops
% Please see example_bw.m if you need an example code.
CC = bwconncomp(BW);

s = regionprops(BW,'centroid');
centroids = cat(1, s.Centroid);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Compute the location of the ball center
%
bw_biggest = zeros(size(pdis));
numPixels = cellfun(@numel,CC.PixelIdxList);
[biggest,idx] = max(numPixels);
bw_biggest(CC.PixelIdxList{idx}) = true; 
segI = bw_biggest;
S = regionprops(CC,'Centroid');
loc = S(idx).Centroid;

  
% Note: In this assigment, the center of the segmented ball area will be considered for grading. 
% (You don't need to consider the whole ball shape if the ball is occluded.)

end
