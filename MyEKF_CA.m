function kalman_filter = MyEKF_CA(kalman_filter,Z,isObjectDetected)
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

% Motion model matrix
F = [1 0 0 dt 0 0 0.5*dt^2 0 0;
     0 1 0 0 dt 0 0 0.5*dt^2 0;
     0 0 1 0 0 dt 0 0 0.5*dt^2;
     0 0 0 1 0 0 dt 0 0;
     0 0 0 0 1 0 0 dt 0;
     0 0 0 0 0 1 0 0 dt;
     0 0 0 0 0 0 1 0 0;
     0 0 0 0 0 0 0 1 0;
     0 0 0 0 0 0 0 0 1];

% Measurement matrix
H = [1 0 0 0 0 0 0 0 0;
     0 1 0 0 0 0 0 0 0;
     0 0 1 0 0 0 0 0 0];


% Extrapolation
X_pred = F * X_est;
P_pred = F * P_est * F.' + Q;

% Check if there was a measurement for correction
if isObjectDetected == 0
    innov = 0;
else
    innov = Z.' - H*X_pred;
end

% Compute the Kalman gain
S = H * P_pred * H.' + R;
K = P_pred * H.' / S;

% Correction step
M = (eye(9) - K*H);
X = X_pred + K*innov;
P = M*P_pred*M.' + K*R*K.';

% Update the kalman_filter structure
kalman_filter.X = X;
kalman_filter.P = P;


end