function movie_maker(fps, output_filename, first_frame,last_frame)
% Define movie settings
format = 'Motion JPEG AVI'; % movie file format

% Create video writer object
v = VideoWriter(output_filename, format);
v.FrameRate = fps;
open(v);

% Loop through all frames and add them to the movie
for i = first_frame:last_frame
    % Load frame from image file
    filename = sprintf('frame_%04d.png', i);
    frame = imread(filename);
    
    % Write frame to movie
    writeVideo(v, frame);
end

% Close video writer object
close(v);
end
