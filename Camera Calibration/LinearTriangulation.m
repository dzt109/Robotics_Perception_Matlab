function X = LinearTriangulation(K, C1, R1, C2, R2, x1, x2)
%% LinearTriangulation
% Find 3D positions of the point correspondences using the relative
% position of one camera from another
% Inputs:
%     C1 - size (3 x 1) translation of the first camera pose
%     R1 - size (3 x 1) rotation of the first camera pose
%     C2 - size (3 x 1) translation of the second camera
%     R2 - size (3 x 1) rotation of the second camera pose
%     x1 - size (N x 2) matrix of points in image 1
%     x2 - size (N x 2) matrix of points in image 2, each row corresponding
%       to x1
% Outputs: 
%     X - size (N x 3) matrix whos rows represent the 3D triangulated
%       points
numpts = size(x1,1);
P1 = K*[R1, C1];
P2 = K*[R2,C2];
x1 = [x1, ones(size(x1,1),1)];
x2 = [x2, ones(size(x2,1),1)];
    
for i=1:numpts
    skew1 = Vec2Skew(x1(i,:)); 
    skew2 = Vec2Skew(x2(i,:));

    A = [skew1*P1; skew2*P2];
    [~,~,V] = svd(A);
    X(i, :) = V(1:3,end)'/V(end,end);

end

function skew = vec2skew(v)
    skew = [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
end
T1 = -R1 * C1;
T2 = -R2 * C2;
P1 = K * [R1 T1];
P2 = K * [R2 T2];
X = zeros(size(x1,1), 3);
for i = 1 : size(x1, 1)
    x1_temp = [x1(i,:), 1]';
    x2_temp = [x2(i,:), 1]';
    x1_temp_skew = vec2skew(x1_temp);
    x2_temp_skew = vec2skew(x2_temp);
    A = [x1_temp_skew * P1 ; x2_temp_skew * P2];
    [u,d,v] = svd(A);
    X_temp = v(:,end) / v(end,end);
    X(i, :) = X_temp(1:3,1)';
end
end


