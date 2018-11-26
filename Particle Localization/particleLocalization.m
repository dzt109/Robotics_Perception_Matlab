% Robotics: Estimation and Learning 
% WEEK 4
% 
% Complete this function following the instruction. 
function myPose = particleLocalization(ranges, scanAngles, map, param)

% Number of poses to calculate
N = size(ranges, 2);
% Output format is [x1 x2, ...; y1, y2, ...; z1, z2, ...]
myPose = zeros(3, N);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Map Parameters 
% 
% % the number of grids for 1 meter.
 myResolution = param.resol;
% % the origin of the map in pixels
 myOrigin = param.origin; 

% The initial pose is given
myPose(:,1) = param.init_pose;
% You should put the given initial pose into myPose for j=1, ignoring the j=1 ranges. 
% The pose(:,1) should be the pose when ranges(:,j) were measured.



% Decide the number of particles, M.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
M = 100;                            % Please decide a reasonable number of M, 
                               % based on your experiment using the practice data.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create M number of particles
P_new = repmat(myPose(:,1), [1, M]);
%P_new = zeros(3,M);
dR = 0.2;
dT = 0.2;
thr = 0.8; %ceil(3/5*size(scanAngles,1))

 for j = 2:N % You will start estimating myPose from j=2 using ranges(:,2).
 
     corr = zeros(1,M);
Weight = ones(1,M)/M;
%     % 1) Propagate the particles 
     P_new(1,:) = P_new(1,:) + dR*cos(P_new(3,:)).*randn(1,M);
     P_new(2,:) = P_new(2,:) + dR*sin(P_new(3,:)).*randn(1,M);
     P_new(3,:) = P_new(3,:) + dT*randn(1,M);
%      
for i = 1:M
    
  mapx = round(P_new(1,i)*param.resol+param.origin(1));
  mapy = round(P_new(2,i)*param.resol+param.origin(2));
  if map(mapy,mapx)>=.49
    continue;
  end  
    
myMAP = zeros(size(map));

%     % 2) Measurement Update 
%     %   2-1) Find grid cells hit by the rays (in the grid map coordinate frame)    
d = ranges(:,j);
lidar_global_x = round((d.*cos(P_new(3,i)+scanAngles) + P_new(1,i))*myResolution + myOrigin(1));
lidar_global_y = round((-d.*sin(P_new(3,i)+scanAngles) + P_new(2,i))*myResolution + myOrigin(2));

    for k=1:size(scanAngles,1)
        if (lidar_global_x(k) >1 && lidar_global_y(k)>1 && lidar_global_x(k)<=size(map,2) && lidar_global_y(k)<=size(map,1))
            sub_indice = sub2ind(size(map), lidar_global_y(k), lidar_global_x(k));
            myMAP(sub_indice)=1;
        end

    end

    %
%     %   2-2) For each particle, calculate the correlation scores of the particles
    corr(i) = sum(sum(map.*myMAP));
 end
%
%     %   2-3) Update the particle weights         
      Weight = Weight.*corr;
      
      Weight = (Weight - min(Weight))/sum(Weight- min(Weight));
      
%     %   2-4) Choose the best particle to update the pose
   
%     
%Weight = Weight(Weight>=thr);
%P_new = P_new(:,Weight >= thr); % select particles with high score
[~,ind] = sort(Weight,'descend');
P_new = P_new(:,ind(1:3));
    
%     % 3) Resample if the effective number of particles is smaller than a threshold

    P_new = repmat(P_new, 1, ceil(M/size(P_new,2)) ); % regenerate particles
    P_new = P_new(:,1:M);
size(P_new)
myPose(:,j) = mean(P_new(:,ind(1:1)),2);

%     % 4) Visualize the pose on the map as needed
%    
% 
 end
%keyboard;
end

