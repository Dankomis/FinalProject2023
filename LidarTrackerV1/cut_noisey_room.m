%%This function will return the point cloud data without the noisey parts
%%that we know caused noise in the lab enviorment

%frame mat - the matrix that holds the LIDAR point cloud data
%x_max - maximum x cor of room
%x_min - minimum x cor of room
%y_max - maximum y cor of room
%y_min - minimum y cor of room

function new_frame = getROI(frame_mat,ROI_flag)

if ROI_flag == 1
    %%Get desired region of intrest
    new_frame = frame_mat(frame_mat(:,1) < x_max & frame_mat(:,1) > x_min & frame_mat(:,2) < y_max & frame_mat(:,2)>y_min,:);
    new_frame = frame_mat(frame_mat(:,1) ~= 0 &frame_mat(:,2)~=0 &frame_mat(:,3)~=0,:);%Ignore al NAN values
else
    new_frame = frame_mat;
end



