function [verif,frame_stream,frame_num] = LidarTrackerV1(data_file,config)
%***********************LIDAR object tracker******************************
%Main tracking system based on using Kalman filter and state estimation in
%each frame using measurement function that detects the tracked object in
%each frame

% Input:
%   - data_file: .csv file that holds the Lidar sensor data output includes
%                x,y,z coordinates and reflectivity value.
%   - config: struct that represents the system's configuration containing the following fields
%           * config.kalman_filter - the type of Kalman filter that will be
%             used
%           * config.meas - the type of detection function that will be
%             used in each frame
%           * config.frame_time - frame duration in seconds
%           * config.frame_points - number of points per frame
%           *config.ROI_flag - flag that indicates if ROI of room should be
%            chosen
%           *config.ROI_flag - flag that indicates if ROI of room should be
%           *config.video_flag - flag that indicates if video stream will
%            be created
% Output:
%   - frame_num: the number of frames in point cloud stream
%   - frame_stream: array that holds frame_num matrices that represent
%                   frame point clout with x,y,z and reflectivity value of each point
%   - verif: struct that holds the output data of the system
%           * verif.estimations - frame_num*sizeof(state_vector) matrix that
%             holds the X state vector for each frame



%%%%%Get system parameters after system was configured%%%%%
%params holds system parameters to be used
params = getParams(config);


%%%%%Data Input Handler%%%%%
%%Handle the LIDAR raw data%% 
%The LIDAR sensor data csv output is loaded and parsed to frames
%Thw csv file is parsed into frames, each frame has frame_points ammount of
%points
[frame_stream,frame_num] = getFrames(data_file,config.frame_points);
params.num_frames = frame_num;%the number of frames also assigned as system param


%%%%%User defined object detection%%%%%
%%Object detection based on user selection%%
%trainig_frame will be the first frame where a user identifies the object and will be prompted to mark it 
trainig_frame = frame_stream{1};
trainig_frame = getROI(trainig_frame,config.ROI_flag,5,0,3,-3);
params.init_location = initPos(trainig_frame);


%%%%%Initialize%%%%%
%%Initialize all system blocks with the relevant parameters%%
%kalman_filter an object that describes the descrete filter
%verif - object that hold system verification data
obj.refAvg = 0;
obj.refVar = 0;
%Initialize according to the applied Kalman filter
switch params.kalman_flag
    case 1 %MyEKF_CV
        kalman_filter.X = zeros(6,1);
        kalman_filter.X(1:3) = params.init_location;
        kalman_filter.P = eye(6);
        kalman_filter.Q = [params.motionNoise(1)^2*eye(3), zeros(3); zeros(3), params.motionNoise(2)^2*eye(3)];
        kalman_filter.R = [params.measNoise^2, 0, 0; 0, params.measNoise^2, 0; 0, 0, params.measNoise^2];
        kalman_filter.dt = params.dt;
        [obj.refAvg,obj.refVar] = get_reflectivityData(trainig_frame,...
            params.init_location(1),...
            params.init_location(2),...
            params.init_location(3),params.object_radius,obj);
        verif.estimations = zeros(params.num_frames,6);
        verif.detections = zeros(params.num_frames,1);
    case 2%MyEKF_CA
        kalman_filter.dt = params.dt;
       [obj.refAvg,obj.refVar] = get_reflectivityData(trainig_frame,...
            params.init_location(1),...
            params.init_location(2),...
            params.init_location(3),params.object_radius,obj);
        kalman_filter.X = zeros(9,1);
        kalman_filter.X(1:3) = params.init_location;
        kalman_filter.P = eye(9);
        % Motion noise covariance matrix
        motionModel1D = [1  kalman_filter.dt  0.5*kalman_filter.dt^2 ;0  1  kalman_filter.dt ;0 0 1];
        G = motionModel1D*[0 0 0;0 0 0;0 0 1]*motionModel1D.';
        kalman_filter.Q = ([G zeros(3) zeros(3);zeros(3) G zeros(3); zeros(3) zeros(3) G])*params.motionNoise^2;
        kalman_filter.R = diag(params.measNoise.^2);
        verif.estimations = zeros(params.num_frames,6);
        verif.detections = zeros(params.num_frames,1);
    case 3 %Matlab Kalman filter ConstantVelocity
        [obj.refAvg,obj.refVar] = get_reflectivityData(trainig_frame,...
            params.init_location(1),...
            params.init_location(2),...
            params.init_location(3),params.object_radius,obj);
        kalman_filter = configureKalmanFilter('ConstantVelocity', ...
            params.init_location,params.initError,params.motionNoise,params.measNoise);
        verif.estimations = zeros(params.num_frames,6);
        verif.detections = zeros(params.num_frames,1);

    case 4 %Matlab Kalman filter ConstantAcceleration
        [obj.refAvg,obj.refVar] = get_reflectivityData(trainig_frame,...
            params.init_location(1),...
            params.init_location(2),...
            params.init_location(3),params.object_radius,obj);
        kalman_filter = configureKalmanFilter('ConstantAcceleration', ...
            params.init_location,params.initError,params.motionNoise,params.measNoise);
        verif.estimations = zeros(params.num_frames,9);
        verif.detections = zeros(params.num_frames,1);
end


%%%%%Frame loop%%%%%
%%Loop all frames and apply the detection on each frame using estimation
%%from previous frame

for i = 1:(frame_num-2)
 
    switch params.kalman_flag
        case 1
            frame_mat = frame_stream{i+1};
            frame_mat = getROI(frame_mat,config.ROI_flag,5,0,3,-3);
            [detectedLocation, isObjectDetected] = get_measIV(frame_mat,kalman_filter.X,params.obj_roi,obj);
            kalman_filter = MyEKF_CV(kalman_filter,detectedLocation, isObjectDetected);
            verif.estimations(i,:) = [kalman_filter.X(1) kalman_filter.X(2) kalman_filter.X(3) kalman_filter.X(4) kalman_filter.X(5) kalman_filter.X(6)];
            verif.detections(i) = isObjectDetected;
            [obj.refAvg,obj.refVar] = get_reflectivityData(frame_mat,...
            kalman_filter.X(1),...
            kalman_filter.X(2),...
            kalman_filter.X(3),params.object_radius,obj);
        case 2
            frame_mat = frame_stream{i+1};
            frame_mat = getROI(frame_mat,config.ROI_flag,5,0,3,-3);
            [detectedLocation, isObjectDetected] = get_measIV(frame_mat,kalman_filter.X,params.obj_roi,obj);
            kalman_filter = MyEKF_CA(kalman_filter,detectedLocation, isObjectDetected);
            verif.estimations(i,:) = [kalman_filter.X(1) kalman_filter.X(2) kalman_filter.X(3) kalman_filter.X(4) kalman_filter.X(5) kalman_filter.X(6)];
            verif.detections(i) = isObjectDetected;
            [obj.refAvg,obj.refVar] = get_reflectivityData(frame_mat,...
            kalman_filter.X(1),...
            kalman_filter.X(2),...
            kalman_filter.X(3),params.object_radius,obj);
        case 3
            frame_mat = frame_stream{i+1};
            X = [kalman_filter.State(1);kalman_filter.State(3);kalman_filter.State(5);kalman_filter.State(2);kalman_filter.State(4);kalman_filter.State(6)];
            [detectedLocation, isObjectDetected] = get_measIV(frame_mat,X,params.obj_roi,obj);
            obj_data = predict(kalman_filter);
            if isObjectDetected == 1
                obj_data = correct(kalman_filter,detectedLocation);
            end
            verif.estimations(i,1:3) = obj_data;
            verif.detections(i) = isObjectDetected;
            [obj.refAvg,obj.refVar] = get_reflectivityData(frame_mat,...
            obj_data(1),...
            obj_data(2),...
            obj_data(3),params.object_radius,obj);

        case 4
            frame_mat = frame_stream{i+1};
            X = [kalman_filter.State(1);kalman_filter.State(4);kalman_filter.State(7);kalman_filter.State(2);kalman_filter.State(5);kalman_filter.State(8);kalman_filter.State(3);kalman_filter.State(6);kalman_filter.State(9)];
            [detectedLocation, isObjectDetected] = get_measIV(frame_mat,X,params.obj_roi,obj);
            obj_data = predict(kalman_filter);
            if isObjectDetected == 1
                obj_data = correct(kalman_filter,detectedLocation);
            end
            verif.estimations(i,1:3) = obj_data;
            verif.estimations(i,4:6) = [kalman_filter.State(2),kalman_filter.State(5),kalman_filter.State(8)];
            verif.detections(i) = isObjectDetected;
            [obj.refAvg,obj.refVar] = get_reflectivityData(frame_mat,...
            obj_data(1),...
            obj_data(2),...
            obj_data(3),params.object_radius,obj);
    end
    
end


