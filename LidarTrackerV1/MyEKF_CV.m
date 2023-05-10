function kalman_filter = MyEKF_CV(kalman_filter,Z,isObjectDetected)
%This function is the Extendent Kalman filter for constant velocity
% Input:
%   - Kalman_filter:In case this filter was chosen it is an object that
%                   holds all relevant fields for using the Kalman filter
%                   method for state estimation
%                   *kalman_filter.X - state vector
%                   *kalman_filter.P = covariance matrix
%                   *kalman_filter.Q = motion model noise covariance matrix
%                   *kalman_filter.R = measurement model noise covariance matrix
%                   *kalman_filter.dt = frame time interval
%   - Z: measurement vector (holds x,y,z values of measured object
%        centroid)
%   - isObjectDetected: flag that mentions if there was a detection and we
%                       get a vaLid Z
%           
% Output:
%   - Kalman_filter:In case this filter was chosen it is an object that
%                   holds all relevant fields for using the Kalman filter
%                   method for state estimation
%                   *kalman_filter.X - state vector
%                   *kalman_filter.P = covariance matrix
%                   *kalman_filter.Q = motion model noise covariance matrix
%                   *kalman_filter.R = measurement model noise covariance matrix
%                   *kalman_filter.dt = frame time interval


%Inits
X_est = kalman_filter.X;
P_est = kalman_filter.P;
Q = kalman_filter.Q;
R = kalman_filter.R;
dt = kalman_filter.dt;


%Predict using motion model
X_pred = [X_est(1) + X_est(4)*dt;
    X_est(2) + X_est(5)*dt;
    X_est(3) + X_est(6)*dt;
    X_est(4);
    X_est(5);
    X_est(6)];

%motion model matrix
F = [1, 0, 0, dt, 0, 0;
    0, 1, 0, 0, dt, 0;
    0, 0, 1, 0, 0, dt;
    0, 0, 0, 1, 0, 0;
    0, 0, 0, 0, 1, 0;
    0, 0, 0, 0, 0, 1];
%measurement matrix
H = [1, 0, 0, 0, 0, 0;
    0, 1, 0, 0, 0, 0;
    0, 0, 1, 0, 0, 0];

%%Covariance Extrapolation%%
 P_pred = F * P_est * F.' + Q;

 %Check if there was a measurement for correction
 if isObjectDetected == 0
     innov = 0;
 else
     innov = Z.' - X_pred(1:3);
 end
 
 
 
 % Compute the Kalman gain
 S = H * P_pred * H.' + R;
 K = P_pred * H.' * inv(S);


 % Update the state and covariance
 M = (eye(6) - K * H);
 X = X_pred + K * innov;
 P =  M*P_pred*M.' + K*R*K.' ;

 kalman_filter.X = X;
 kalman_filter.P = P;
end



