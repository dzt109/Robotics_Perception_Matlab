function [C, R] = LinearPnP(X, x, K)
%% LinearPnP
% Getting pose from 2D-3D correspondences
% Inputs:
%     X - size (N x 3) matrix of 3D points
%     x - size (N x 2) matrix of 2D points whose rows correspond with X
%     K - size (3 x 3) camera calibration (intrinsics) matrix
% Outputs:
%     C - size (3 x 1) pose transation
%     R - size (3 x 1) pose rotation
%
% IMPORTANT NOTE: While theoretically you can use the x directly when solving
% for the P = [R t] matrix then use the K matrix to correct the error, this is
% more numeically unstable, and thus it is better to calibrate the x values
% before the computation of P then extract R and t directly

numpts = size(x,1);
Xdiag = zeros(3,12);
A = [];

 x = [x, ones(size(x,1),1)];
 X = [X, ones(size(X,1),1)];
 
for i=1:numpts
    Xdiag(1, 1:4) = X(i,:);
    Xdiag(2, 5:8) = X(i,:);
    Xdiag(3, 9:12) = X(i,:);
    skew = Vec2Skew(inv(K)*x(i,:)');
    A = [A; skew*Xdiag];

end

[~,~,V] = svd(A);
Pvec = V(:,end);
P = [Pvec(1:4)';Pvec(5:8)';Pvec(9:12)'];

% check
check = zeros(3, numpts);
for i = 1:numpts
    check(:, i) = Vec2Skew(inv(K)*x(i,:)')*P*X(i,:)';
end

%reconditioning
RC = P;
[U,D,V] = svd(RC(:,1:3));
R = sign(det(U*V'))*U*V';
C = -R'*RC(:,4)/D(1,1);

end





