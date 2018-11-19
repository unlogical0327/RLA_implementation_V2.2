%%%%%%%%%%%
% Test script
%%%%%%%%%%%%%%%%%%%%%%%%%%
clear;
% --Define algorithm parameters here
interrupt=0;
scan_freq=2;   % set scan frequency from 8 up to 30Hz, measurement mode will do the loop and send back the data
%%reflector_map='Reflector_map';
reflector_source_flag=2;    % set the list flag to 0--read from file, 1--manually set the reflector location 2--generate 120x110 reflector array 2--generate from random location
%data_source_flag=1;
req_update_match_pool=0;
num_ref_pool=4;
num_detect_pool=3;
%Reflector_map=0;
scan_data=0;
amp_thres=1500;
reflector_diameter=100;
dist_delta=200;
dist_thres=200;  % minimum distance to recognize a reflector
thres_dist_large=8000;   % this is the filter to knock out the large distance points
thres_dist_match=30;   % distance threshold to match reflectors
thres_angle_match=1;  % angle threshold to match reflectors
% Execute mode manager to run GUI test script
[mode,status,update_match_pool]=mode_manager(interrupt,scan_freq,reflector_source_flag,req_update_match_pool,num_ref_pool,num_detect_pool,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,thres_dist_match,thres_dist_large,thres_angle_match);

if status == 0
    disp('RLA is running properly!!')
elseif status == 1
    error('Error happen!!')
end
