function fit_box(frame_mat,obj_info,delta,i,video_flag)

x = frame_mat(:,1);
y = frame_mat(:,2);
z = frame_mat(:,3);

% Compute Euclidean distances between tracked_object and detected_objects
roi = frame_mat((x > (obj_info(:,1) - delta) & x < (obj_info(:,1) + delta) & y > (obj_info(:,2) - delta) & y < (obj_info(:,2) + delta) & z > (obj_info(:,3) - delta )& z < (obj_info(:,3) + delta)),1:3);

if isempty(roi)
    figure
    pcshow(frame_mat(:,1:3),frame_mat(:,4),'MarkerSize', 50);
    hold on
    title(["Detection failed for frame: " ,num2str(i)])
else
    % Find the index of the detected object with the smallest distance
    distances = sqrt(sum((roi - obj_info).^2, 2));
    [min_distance, object_index] = min(distances);
    obj_point = roi(object_index,:);
    obj_ix = find(x > (obj_point(:,1) - delta) & x < (obj_point(:,1) + .1) & y > (obj_point(:,2) - .1) & y < (obj_point(:,2) + .1) & z > (obj_point(:,3) - .1 )& z < (obj_point(:,3) + .1));
    pcd = pointCloud(frame_mat(:,1:3));
    test_fit = pcfitcuboid(pcd,obj_ix);
    if video_flag == 1
        fig = figure('Visible', 'off');
        pcshow(frame_mat(:,1:3),frame_mat(:,4),'MarkerSize', 50);
        hold on
        plot(test_fit);
        %save frame as image file
        filename=sprintf('frame_%04d.png',i);
        saveas(fig,filename);
        % close the figure to clean up
        close(fig);
    else
        figure;
        pcshow(frame_mat(:,1:3),frame_mat(:,4),'MarkerSize', 50);
        hold on
        plot(test_fit);
        title(["Detection for frame: " ,num2str(i)])

        
    end
    
end





