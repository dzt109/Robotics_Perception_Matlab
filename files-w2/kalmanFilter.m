function [ predictx, predicty, state, param ] = kalmanFilter( t, x, y, state, param, previous_t )
%UNTITLED Summary of this function goes here
%   Four dimensional state: position_x, position_y, velocity_x, velocity_y

    %% Place parameters like covarainces, etc. here:
     P = eye(4)
     R = eye(2)
    spx = 0.1;
    spy = 0.1;
    svx = 10;
    svy = 10;
    Ss = diag([spx,spy,svx,svy]);  %System noise covariance
    Sm = diag([spx,spy]);  %Measurement noise covariance
    
    
    
    % Check if the first time running this function
    if previous_t<0
        state = [x, y, 0, 0];
        param.P = 0.1 * eye(4)*1000000;
        predictx = x;
        predicty = y;
        return;
    end
    P = param.P;
    %% TODO: Add Kalman filter updates
    % As an example, here is a Naive estimate without a Kalman filter
    % You should replace this code
    dt = 0.033;
    A = [1 0 dt 0;0 1 0 dt;0 0 1 0;0 0 0 1];
    B = zeros(2);
    u = zeros(2,1);
    C = [1,0,0,0;0,1,0,0];
    
    
    P = A*param.P*A' + Ss;  %System
    R = 0*C*P*C' + Sm;  %Measurement
    
    K=P*C'*inv(R+C*P*C')  %Gain
    %keyboard;
    
    temp=A*state'+K*([x;y]-C*A*state');
    vx=temp(3);
    vy=temp(4);
    predictx=temp(1)+vx*10*dt
    predicty=temp(2)+vy*10*dt
    [predictx, predicty, x,y];
    param.P=P-K*C*P;
    % State is a four dimensional element
    %state=temp';
    state = [x, y, vx, vy];
    
    % State is a four dimensional element
    state = temp';
end
