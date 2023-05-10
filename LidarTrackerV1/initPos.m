function position = initPos(frame_mat)
 %This function should get the frame point cloud data
 % it enables the user to choose the position of the obsticle at the
 %first frame
 % Input:
 %   - frame_mat: Nx4 matrix where N is the number of points in the point cloud.
 %                  The first three columns represent the x, y, and z coordinates of each point,
 %                  and the fourth column represents the reflectivity of each point.
 % Output:
 %   - Position: location of detected object
 

    % Extract the x, y, z, and reflectivity data from the frame
    x = frame_mat(:,1);
    y = frame_mat(:,2);
    z = frame_mat(:,3);
    reflectivity = frame_mat(:,4);
    figure;
    % Plot only the points that meet the condition
    scatter3(x, y, z, 10, reflectivity, 'filled');
    colorbar;
    xlabel('X');
    ylabel('Y');
    zlabel('Z');
    
    % Enable data cursor mode
    dcm_obj = datacursormode(gcf);
    set(dcm_obj, 'DisplayStyle', 'datatip', 'SnapToDataVertex', 'on', 'Enable', 'on');
    
    % Initialize myValue to NaN
    %persistent myValue;
    % Define a callback function to update myValue when the user clicks on a point
    set(dcm_obj, 'UpdateFcn', @select_point_callback);
    
    % Allow the user to rotate the figure using the mouse
    rotate3d on;
    
    % Wait for user to select a point using the data cursor
    title('Please select an existing point...');
    
    % Wait for user to press Enter
    fprintf('Please select an existing point...\n');
    fprintf('rotate the figure for checking the position\n');
    fprintf('if the position is wrong - select other point and check again\n');
    fprintf('Press Enter to continue...\n');
    pause;
    
    % Get the updated value of myValue from the persistent variable inside the callback function
    myValue = getappdata(gcf, 'myValue');
    position=myValue;
    % Close the figure
    close;
   
    % Define the select_point_callback function
    function output_txt = select_point_callback(~, event_obj)
    % This function is called whenever the user selects a point using the data cursor
    % Declare the persistent variable to store myValue
       % Get the x and y coordinates of the selected point
        x = event_obj.Position(1);
        y = event_obj.Position(2);
        z = event_obj.Position(3);
        % Update the value of myValue
        myValue = [x y z];
        % Store myValue in the persistent variable
        setappdata(gcf, 'myValue', myValue);
    
        % Display the selected point
        hold on;
        scatter3(x, y, z, 'r', 'filled');
        title(sprintf('Selected point: (%f,%f,%f)', x, y, z));
        drawnow;
        
        % Return xyz output
        output_txt = [x,y,z];
    end
end
