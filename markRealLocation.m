%Get real position values using user location marker on point cloud

% pcd_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\14.3.23\14.3 csv\2balls_dyn_res_n.csv";
pcd_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\1_ball_open_field.csv";
outFile = "C:\Users\DanK\Desktop\1BallOpenFieldReal.xlsx";
[frame_stream,frame_num] = getFrames(pcd_file,config.frame_points);

tracked_pts = zeros(20,3);
idx = 1;
for i = 5:5:100
    frame = frame_stream{i};
    frame = getROI(frame,1,5,0,3,-3);
    tracked_pts(idx,:) = initPos(frame);
    idx = idx +1;
end

writematrix(tracked_pts,outFile);