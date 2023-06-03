%%%%%%%%%%%%Testing program for Lidar Tracker%%%%%%%%%%%%%%%%%%%%%
%%%This file simulates the tracking system under different situations and
%%%It configures the system and outputs stream,frame detection and
%%%different success matrices


%%%Real Location files for validation%%%
%%The files contains manual samples of the object's location in time
realData_2ballsRoom_file = "C:\Users\DanK\Desktop\פרויקט גמר\Final project matlab\Real locations\2BallsRoomRealData.xlsx";
realData_1ballField_file = "C:\Users\DanK\Desktop\פרויקט גמר\Final project matlab\Real locations\1BallOpenFieldReal.xlsx";
realData_1ballRoom_file = "C:\Users\DanK\Desktop\פרויקט גמר\Final project matlab\Real locations\1BallRoomReal.xlsx";
realData_1ballWall_file = "C:\Users\DanK\Desktop\פרויקט גמר\Final project matlab\Real locations\1ballWall.xlsx";


realData_1ballField = readmatrix(realData_1ballField_file);
realData_2ballsRoom = readmatrix(realData_2ballsRoom_file);
realData_1ballRoom = readmatrix(realData_1ballRoom_file);
realData_1ballWall = readmatrix(realData_1ballWall_file);

%%Simulations: in this section the system will be simulated with
%%%%different data and configurations

simType = 4;%flag that indicates what kind of simulation will be executed

switch simType
     case 1 %Simulate and Create frame detection stream
        %configure the tracking system
        config.kalman_filter = 4;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 1;
        [verif,frame_stream,frame_num] = LidarTrackerV1(data_file,config);

        %%Create video
        for j = 1:1:100
            frame_j = frame_stream2{j+1};
            if config.ROI_flag == 1
                frame_j = getROI(frame_j,config.ROI_flag,5,0,3,-3);
            end
            fit_box(frame_j,verif2.estimations(j,1:3),0.5,j,1);
        end
        test_movie = "frame_stream_test.avi";
        movie_maker(1/config.frame_time,test_movie, 1,100);

     case 2 %Simulate and display frame detection
        data_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\1_ball_open_field.csv";
        %configure the tracking system
        config.kalman_filter = 4;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif,frame_stream,frame_num] = LidarTrackerV1(data_file,config);

        for j = 10:10:100
            frame_j = frame_stream{j+1};
            if config.ROI_flag == 1
                frame_j = getROI(frame_j,config.ROI_flag,5,0,3,-3);
            end
            fit_box(frame_j,verif.estimations(j,1:3),0.5,j,config.video_flag);
        end

    case 3 %Simulate + info 1 ball open field
        data_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\1_ball_open_field.csv";
        real_data = realData_1ballField;
        %configure the tracking system
        config.kalman_filter = 4;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif,frame_stream,frame_num] = LidarTrackerV1(data_file,config);

        %%%Location%%%
        % create figure and three subplots
        calculated_data = verif.estimations(:,1:3);
        figure
        title("Object detection")
        subplot(3, 1, 1)
        hold on
        title('Location in X')
        xlabel('Frame number sps = 2' )
        ylabel('Position (m)')
        plot(1:20, real_data(:, 1), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 1), 'r', 'DisplayName', 'Calculated location')
        legend

        subplot(3, 1, 2)
        hold on
        title('Location in Y')
        xlabel('Frame number sps = 2')
        ylabel('Position (m)')
        plot(1:20, real_data(:, 2), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 2), 'r', 'DisplayName', 'Calculated location')
        legend

        subplot(3, 1, 3)
        hold on
        title('Location in Z')
        xlabel('Frame number sps = 2')
        ylabel('Position (m)')
        plot(1:20, real_data(:, 3), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 3), 'r', 'DisplayName', 'Calculated location')
        legend


        %%%Velocity%%%
        % create figure and three subplots
        calculated_data = verif.estimations(:,4:6);
        figure
        title("Object's Velocity")
        subplot(3, 1, 1)
        hold on
        title('Velocity in X')
        xlabel('Frame number sps = 2' )
        ylabel('Velocity (m/s)')
        plot(1:20, calculated_data(5:5:100, 1), 'r', 'DisplayName', 'Calculated velocity')

        subplot(3, 1, 2)
        hold on
        title('Velocity in Y')
        xlabel('Frame number sps = 2')
        ylabel('Velocity (m/s)')
        plot(1:20, calculated_data(5:5:100, 2), 'r', 'DisplayName', 'Calculated velocity')

        subplot(3, 1, 3)
        hold on
        title('Velocity in Z')
        xlabel('Frame number sps = 2')
        ylabel('Velocity (m/s)')
        plot(1:20, calculated_data(5:5:100, 3), 'r', 'DisplayName', 'Calculated velocity')

    case 4 
    %%Simulate different Kalman filters and conclude reults
        %%The system will be simulated with 1 ball in open field
        data_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\1_ball_open_field.csv";
        real_data = realData_1ballField;
        %Simulation1: Matlab Kalman filter constant velocity 
        config.kalman_filter = 3;%configure the tracking system
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif1,frame_stream1,frame_num1] = LidarTrackerV1(data_file,config);

        %Simulation2: Matlab Kalman filter constant accelaration
        config.kalman_filter = 4;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif2,frame_stream2,frame_num2] = LidarTrackerV1(data_file,config);

        %%Simulation3: EKF filter constant velocity
        config.kalman_filter = 1;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif3,frame_stream3,frame_num3] = LidarTrackerV1(data_file,config);
        
       
        %%Compare convergence of different simulations
        calculated_data1 = verif1.estimations(:,1:3);
        calculated_data2 = verif2.estimations(:,1:3);
        calculated_data3 = verif3.estimations(:,1:3);
        
        % calculate Euclidean distance between real and calculated locations for
        % each simulation

        dist1 = sqrt((real_data(:,1) - calculated_data1(5:5:100,1)).^2 + ...
            (real_data(:,2) - calculated_data1(5:5:100,2)).^2 + ...
            (real_data(:,3) - calculated_data1(5:5:100,3)).^2 );

        dist2 = sqrt((real_data(:,1) - calculated_data2(5:5:100,1)).^2 + ...
            (real_data(:,2) - calculated_data2(5:5:100,2)).^2 + ...
            (real_data(:,3) - calculated_data2(5:5:100,3)).^2 );

        dist3 = sqrt((real_data(:,1) - calculated_data3(5:5:100,1)).^2 + ...
            (real_data(:,2) - calculated_data3(5:5:100,2)).^2 + ...
            (real_data(:,3) - calculated_data3(5:5:100,3)).^2 );
        % calculate mean and standard deviation of distances
        mean_dist1 = mean(dist1);
        std_dist1 = std(dist1);

        mean_dist2 = mean(dist2);
        std_dist2 = std(dist2);

        mean_dist3 = mean(dist3);
        std_dist3 = std(dist3);

        % plot distance as a function of frame number
        figure
        hold
        plot(1:20, dist1, 'r', 'LineWidth', 2)
        plot(1:20, dist2, 'g', 'LineWidth', 2)
        plot(1:20, dist3, 'b', 'LineWidth', 2)
        title('Euclidean distance between real and calculated locations')
        xlabel('Frame number sps = 2')
        ylabel('Distance (m)')
        legend('constant velocity', 'constant accelaration', 'EKF')

        case 5 
        %%Simulate under multiple object env.
        %%The system will be simulated with 2 ball in dark room
        %%Right ball was marked
        %configure the tracking system
        data_file = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\14.3.23\14.3 csv\2balls_dyn_res_n.csv";
        real_data = realData_1ballWall;

        config.kalman_filter = 4;
        config.meas = 2;
        config.frame_time = 0.1;
        config.object_radius = 0.1;

        config.frame_points = 24000;
        config.ROI_flag = 1 ;
        config.video_flag = 0;
        [verif,frame_stream,frame_num] = LidarTrackerV1(data_file,config);

        % create figure and three subplots
        calculated_data = verif.estimations(:,1:3);
        figure
        title("Object detection")
        subplot(3, 1, 1)
        hold on
        title('Location in X')
        xlabel('Frame number sps = 2' )
        ylabel('Position (m)')
        plot(1:20, real_data(:, 1), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 1), 'r', 'DisplayName', 'Calculated location')
        legend

        subplot(3, 1, 2)
        hold on
        title('Location in Y')
        xlabel('Frame number sps = 2')
        ylabel('Position (m)')
        plot(1:20, real_data(:, 2), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 2), 'r', 'DisplayName', 'Calculated location')
        legend

        subplot(3, 1, 3)
        hold on
        title('Location in Z')
        xlabel('Frame number sps = 2')
        ylabel('Position (m)')
        plot(1:20, real_data(:, 3), 'b', 'DisplayName', 'Real location')
        plot(1:20, calculated_data(5:5:100, 3), 'r', 'DisplayName', 'Calculated location')
        legend


        case 6
            %%Simulate noisey env
            %Simulation1: Medium SNR
            %%The system will be simulated with 1 ball in open field
            data_file1 = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\1_ball_open_field.csv";
            real_data1 = realData_1ballField;
            
            config.kalman_filter = 4;%configure the tracking system
            config.meas = 2;
            config.frame_time = 0.1;
            config.object_radius = 0.1;

            config.frame_points = 24000;
            config.ROI_flag = 1 ;
            config.video_flag = 0;
            [verif1,frame_stream1,frame_num1] = LidarTrackerV1(data_file1,config);

            %Simulation2: Matlab Kalman filter constant accelaration
            data_file2 = "C:\Users\DanK\Desktop\פרויקט גמר\Recordings\3.4.23\csv\ball_wall.csv";
            real_data2 = realData_1ballField;
            config.kalman_filter = 4;
            config.meas = 2;
            config.frame_time = 0.1;
            config.object_radius = 0.1;

            config.frame_points = 24000;
            config.ROI_flag = 1 ;
            config.video_flag = 0;
            [verif2,frame_stream2,frame_num2] = LidarTrackerV1(data_file2,config);

            
            %%Compare convergence of different simulations
            calculated_data1 = verif1.estimations(:,1:3);
            calculated_data2 = verif2.estimations(:,1:3);
            

            % calculate Euclidean distance between real and calculated locations for
            % each simulation

            dist1 = sqrt((real_data1(:,1) - calculated_data1(5:5:100,1)).^2 + ...
                (real_data1(:,2) - calculated_data1(5:5:100,2)).^2 + ...
                (real_data1(:,3) - calculated_data1(5:5:100,3)).^2 );

            dist2 = sqrt((real_data2(:,1) - calculated_data2(5:5:100,1)).^2 + ...
                (real_data2(:,2) - calculated_data2(5:5:100,2)).^2 + ...
                (real_data2(:,3) - calculated_data2(5:5:100,3)).^2 );

            % calculate mean and standard deviation of distances
            mean_dist1 = mean(dist1);
            std_dist1 = std(dist1);

            mean_dist2 = mean(dist2);
            std_dist2 = std(dist2);

            
            % plot distance as a function of frame number
            figure
            hold
            plot(1:20, dist1, 'r', 'LineWidth', 2)
            plot(1:20, dist2, 'g', 'LineWidth', 2)
           
            title('Euclidean distance between real and calculated locations')
            xlabel('Frame number sps = 2')
            ylabel('Distance (m)')
            legend('Average SNR', 'Low SNR')


end