function F = EstimateFundamentalMatrix(x1, x2)
%% EstimateFundamentalMatrix
% Estimate the fundamental matrix from two image point correspondences 
% Inputs:
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Output:
%    F - size (3 x 3) fundamental matrix with rank 2

yt = [];
for i=1:size(x1,1)
    y1 = x1(i,1); 
    y1p = x2(i,1);
    y2= x1(i,2);
    y2p= x2(i,2);
    yt(i,:) = [y1p*y1, y1p*y2,y1p,y2p*y1,y2p*y2,y2p,y1,y2,1];
    

end

[U,S,V] = svd(yt);
f = V(:,end);
f1 = reshape(f,[3,3]);

[u,s,v] = svd(f1);
s(end,end) = 0;
f1 = u*s*v';
F = f1/norm(f1);
%keyboard;



