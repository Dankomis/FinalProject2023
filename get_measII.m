function [detectedLocation, isObjectDetected] = get_measII(frame_mat,X,delta)
 % Input:
 %   - frame_mat: Nx4 matrix where N is the number of points in the point cloud.
 %                  The first three columns represent the x, y, and z coordinates of each point,
 %                  and the fourth column represents the reflectivity of each point.
 %   - X: the state vector, holds the x,y,z coordinates and the velocity
 %        for each coordinate
 %   - delta:size using to configure the region of interest for detection
 % Output:
 %   - detectedLocation: location of detected object
 %   -isObjectDetected: flag indicates if object was detected in the frame

%%Intialize
isObjectDetected = 1;%initialize object to be detected by default
tracked_object = [X(1) X(2) X(3)];%Use state estimation to define tracked object

% Define an ROI
roi.x_min = tracked_object(1,1) - delta;
roi.x_max = tracked_object(1,1) + delta;
roi.y_min = tracked_object(1,2) - delta;
roi.y_max = tracked_object(1,2) + delta;
roi.z_min = tracked_object(1,3) - delta;
roi.z_max = tracked_object(1,3) + delta;

[objects,objectsFound] = detect_objects(frame_mat, roi); %find all objects in ROI

if objectsFound == 0 %check if any objects where found
    detectedLocation = [];
    isObjectDetected = 0;
else

    %Get centroids of each cluster
    detected_objects = zeros(length(objects),3);
    for j = 1:length(objects)
        detected_objects(j,1) = mean(objects{j}(:,1));
        detected_objects(j,2) = mean(objects{j}(:,2));
        detected_objects(j,3) = mean(objects{j}(:,3));
    end

    [detectedLocation, distances] = nearest_objects(tracked_object, detected_objects,0.5);% find nearest object to the estimated location

    %Check if object was detected
    empty_check = isempty(detectedLocation);
    if (empty_check == 1)
        isObjectDetected = 0;
    else
        nan_check = isnan(detectedLocation);
        if ((nan_check(1)|nan_check(2)|nan_check(3)) ~= 0)
            isObjectDetected = 0;
        end

    end
end

end


%%%%%%%Data association blocks%%%%%%%

function [objects,objectsFound] = detect_objects(point_cloud, roi)
    % Input:
    %   - point_cloud: Nx4 matrix where N is the number of points in the point cloud. 
    %                  The first three columns represent the x, y, and z coordinates of each point, 
    %                  and the fourth column represents the reflectivity of each point.
    %   - roi: struct containing the following fields:
    %           * x_min: minimum x value of the region of interest
    %           * x_max: maximum x value of the region of interest
    %           * y_min: minimum y value of the region of interest
    %           * y_max: maximum y value of the region of interest
    %           * z_min: minimum z value of the region of interest
    %           * z_max: maximum z value of the region of interest
    % Output:
    %   - objects: cell array containing the detected objects. Each cell in the array represents an object 
    %              and is a matrix where each row corresponds to a point in the object.
    %   -objectsFound: flag indicates if cany objects where found in the
    %                  ROI
    
    objectsFound = 1; %assume objects are found
    % Extract x, y, z, and reflectivity columns from point cloud matrix
    x = point_cloud(:, 1);
    y = point_cloud(:, 2);
    z = point_cloud(:, 3);
    reflectivity = point_cloud(:, 4);

    % Find points that are within the ROI
%     roi_indices = find(x >= roi.x_min & x <= roi.x_max & ...
%                        y >= roi.y_min & y <= roi.y_max & ...
%                        z >= roi.z_min & z <= roi.z_max & reflectivity > 10 & reflectivity < 70);
    
    roi_indices = find(x >= roi.x_min & x <= roi.x_max & ...
        y >= roi.y_min & y <= roi.y_max & ...
        z >= roi.z_min & z <= roi.z_max & reflectivity > 0);
    roi_points = point_cloud(roi_indices,:);
    reflectivityAvg = mean(reflectivity(roi_indices));
    reflectivityVar = var(reflectivity(roi_indices));
    points = roi_points(abs(reflectivity(roi_indices) - reflectivityAvg) < sqrt(reflectivityVar),1:3); 

    if size(roi_indices,1) == 0
        objects = cell(1, 1);
        objectsFound = 0;
    else
        %points = [x(roi_indices), y(roi_indices), z(roi_indices)];
        min_distance = 0.15; % minimum distance between points in a cluster
        min_points = 5; % minimum number of points in a cluster
        IDX = dbscan(points, min_distance, min_points);
        num_objects = max(IDX);
        if (num_objects < 0)
            objects = cell(1, 1);
            object = points;
            objects{1} = object;
        else% Cluster points using Euclidean distance
            % Extract clusters as objects
            objects = cell(1, num_objects);
            for i = 1:num_objects
                %object_indices = roi_indices(IDX == i);
                %object = [x(object_indices), y(object_indices), z(object_indices), reflectivity(object_indices)];
                object = points(IDX == i,1:3);
                objects{i} = object;

            end
        end
    end    
end


function [object_centroid, min_distance] = nearest_objects(tracked_object, detected_objects,max_dist)
    % tracked_object: 1x3 array representing the position of the tracked object
    % detected_objects: Nx3 array representing the positions of the detected objects
    % Compute Euclidean distances between tracked_object and detected_objects
    distances = sqrt(sum((detected_objects - tracked_object).^2, 2));
    % Find the index of the detected object with the smallest distance
    [min_distance, object_index] = min(distances);
    if min_distance > max_dist
        object_centroid = [nan nan nan];
        min_distance = [];
    else
        object_centroid = detected_objects(object_index,:);
    end


    
end