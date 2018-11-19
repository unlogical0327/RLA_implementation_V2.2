% continuous loading Lidar data to test moving algorithm
%--function returns lidar data, the length of data and how many round of
%--scan data in file
%-- mode = cali, read the first round data
%-- mode = meas, read the second round data
%-- mode = moving, read the data repeatly to the end
function [Lidar_data,data_length,data_round]=load_continous_scan_data(fname,mode)
    Lidar_data_total = dlmread( fname, '\t', 3, 0)';
    pose_180=find(Lidar_data_total(1,:)==-180);
    data_length=pose_180(1,2)-pose_180(1,1);
    if mode=='cali'
    Lidar_data = Lidar_data_total(:,1:data_length);
    data_round=1;
    elseif mode=='meas'
    Lidar_data = Lidar_data_total(:,data_length+1:2*data_length);   
    data_round=2;
    elseif mode=='movi'
    Lidar_data = Lidar_data_total(:,2*data_length+1:end);   
    data_round=length(pose_180)-2;
    end