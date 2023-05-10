function [kalman_filter,verif] = init(params)
%%This function will Initialize:
%%X - state vector
%%P - covariance matrix
%%verif - output data object

%Initialize according to the applied Kalman filter
switch params.kalman_flag
    case 1
        kalman_filter.X = zeros(6,1);
        kalman_filter.P = eye(6);
        kalman_filter.Q = [params.motionNoise(1)^2*eye(3), zeros(3); zeros(3), params.motionNoise(2)^2*eye(3)];
        kalman_filter.R = [params.measNoise^2, 0, 0; 0, params.measNoise^2, 0; 0, 0, params.measNoise^2];
        kalman_filter.dt = params.dt;
        verif.estimations = zeros(params.num_frames,6);
    case 2
        kalman_filter.X = zeros(6,1);
        kalman_filter.P = eye(6);
        verif.estimations = zeros(params.num_frames,6);
    case 3
        kalman_filter.X = zeros(9,1);
        kalman_filter.P = eye(9);
        verif.estimations = zeros(params.num_frames,9);
end


