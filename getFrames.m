function [frame_stream,frame_num] = getFrames(data_file,frame_points)
%%This function will parse the LIDAR data stream into frames
% Input:
%   - data_file: .csv file that holds the Lidar sensor data output includes
%                x,y,z coordinates and reflectivity value.
%   - frame_points - number of points per frame
%           
% Output:
%   - frame_num: the number of frames in point cloud stream
%   - frame_stream: array that holds frame_num matrices that represent
%                   frame point clout with x,y,z and reflectivity value of each point


%get all frames in the x,y,z,reflectivity format
[frames_mat,numRows] = get_frame(data_file,1);
frame_num = ceil(numRows/frame_points);
frame_stream = cell(1,frame_num);
%parse the data into frames
frame_stream{1} = frames_mat(1:24000,:);%get first frame
for i=1:(frame_num-2)
    frame_stream{i+1} = frames_mat(1+ i*frame_points:(i+1)*frame_points,:);%save the other frames into the stream
end

end