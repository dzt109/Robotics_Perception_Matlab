% Robotics: Estimation and Learning 
% WEEK 3
% 
% Complete this function following the instruction. 
function myMap = occGridMapping(ranges, scanAngles, pose, param)


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
% Parameters 
% 
% % the number of grids for 1 meter.
 myResol = param.resol;
% % the initial map size in pixels
 myMap = zeros(param.size);
% % the origin of the map in pixels
 myorigin = param.origin; 
% 
% % 4. Log-odd parameters 
 lo_occ = param.lo_occ;
 lo_free = param.lo_free; 
 lo_max = param.lo_max;
 lo_min = param.lo_min;

 N = size(pose,2);
 for j = 1:N % for each time,
% 
%       
%     % Find grids hit by the rays (in the gird map coordinate)

x = pose(1,j);
y = pose(2,j);

theta = pose(3,j);
d = ranges(:,j);
alpha = scanAngles;
x_occ = d.*cos(theta+alpha) + x;
y_occ = -d.*sin(theta+alpha) + y;

i_occ = ceil(myResol*x_occ)+myorigin(1);
j_occ = ceil(myResol*y_occ)+myorigin(2);

i_pose = ceil(myResol*x)+myorigin(1);
j_pose = ceil(myResol*y)+myorigin(2);
%   

%     % Find occupied-measurement cells and free-measurement cells
for i = 1:length(i_occ)
[freex, freey] = bresenham(i_pose,j_pose,i_occ(i),j_occ(i));  

%     % Update the log-odds
%   
    myMap(freex,freey) = max(myMap(freex,freey) - lo_free,lo_min);
    myMap(i_occ,j_occ) =min(myMap(i_occ,j_occ) + lo_occ,lo_max);
end
% 
%     % Saturate the log-odd values
%     
% 
%     % Visualize the map as needed
%    
% 
%imshow(myMap);
% end

end

