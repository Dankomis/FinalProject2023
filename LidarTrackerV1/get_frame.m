function [frame_mat,numRows] = get_frame(csv_file,start)
%%This function will save the LIDAR recording data into a matrix
% Input:
%   - data_file: .csv file that holds the Lidar sensor data output includes
%                x,y,z coordinates and reflectivity value.
%   - start - first point start index
%           
% Output:
%   - numRows: the number of frames in point cloud stream
%   - frame_mat: matrix that hold all stream data

%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 19);

% Specify range and delimiter
opts.DataLines = [start, Inf];
opts.Delimiter = ",";

% Specify column names and types
opts.VariableNames = ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8", "X", "Y", "Z", "Reflectivity", "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"];
opts.SelectedVariableNames = ["X", "Y", "Z", "Reflectivity"];
opts.VariableTypes = ["string", "string", "string", "string", "string", "string", "string", "string", "double", "double", "double", "double", "string", "string", "string", "string", "string", "string", "string"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8",  "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"], "WhitespaceRule", "preserve");
opts = setvaropts(opts, ["Var1", "Var2", "Var3", "Var4", "Var5", "Var6", "Var7", "Var8",  "Var13", "Var14", "Var15", "Var16", "Var17", "Var18", "Var19"], "EmptyFieldRule", "auto");

% Import the data
frame_mat = readtable(csv_file, opts);
numRows = height(frame_mat);
%% Convert to output type
frame_mat = table2array(frame_mat);

%% Clear temporary variables
clear opts