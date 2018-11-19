%% Measurement mode module
% -- this is the measurement mode module used to conduct the continuing
% measurement when Lidar starts to measure the location
function [mea_status,Lidar_trace,rotation_trace,Lidar_update_Table,match_reflect_pool,matched_reflect_ID,reflector_index,detected_reflector2,detected_ID2] = measurement_mode(num_ref_pool_orig,num_detect_pool,Reflector_map,Reflector_map_polar,Reflector_ID,measurement_data3,scan_data,amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_trace,rotation_trace,thres_dist_match,thres_dist_large,thres_angle_match)
%% 1. Read the scan data, identify reflectors and define how many scanned reflectors are used from the list(nearest distance or most distingushed).
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% identify the reflectors
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
measurement_data(:,1)=measurement_data3(:,1);
measurement_data(:,2)=measurement_data3(:,2);
[detected_ID,detected_reflector,reflector_index]=identify_reflector(amp_thres,dist_thres,reflector_diameter,dist_delta,measurement_data,scan_data);
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
num_ref_pool=num_ref_pool_orig;
Lidar_x=0;
Lidar_y=0;
Lidar_current_xy=[0 0];
theta_rot=0;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% For the case less than 3 reflectors detected.
if length(detected_ID)<3
    disp('detected reflectors smaller than 2....')
    %-- pass the null value to output
    reflector_rmse=99.99;
    Lidar_trace=[0 0];
    rotation_trace=0;
    Lidar_update_Table=0;
    detected_ID2=0;
    detected_reflector2=0;
    match_reflect_pool=0;
    matched_reflect_ID=0;
else
%% 2. Match the N x scanned reflectors with match reflector table and find the location of lidar.
% Detect if pool size is set to less than 3
if num_detect_pool<=2
    disp('detected reflector is defined a illegal reflector number to RLA. automatically set to 3 to start from....')
    num_detect_pool=3;
end
if num_ref_pool<=2
    disp('Reference reflector is defined a illegal reflector number to RLA. automatically set to 3 to start from....')
    num_ref_pool=3;
end
% -- create match detect pool
if num_ref_pool<=length(Reflector_map)
[match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_current_xy);
elseif num_ref_pool>length(Reflector_map)
 num_ref_pool=length(Reflector_map);
[match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_current_xy);
end
if num_detect_pool<=length(detected_reflector)
    [match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(num_detect_pool,detected_reflector,detected_ID,Lidar_current_xy);
elseif num_detect_pool>length(detected_reflector)
    num_detect_pool=length(detected_reflector);
    [match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(num_detect_pool,detected_reflector,detected_ID,Lidar_current_xy);
end
%% 2.a Calculate distance between any two reflectors
%% 2.b match detected reflectors with match reflector pool and return matched point ID.
% -- match the distance vector and return point array and point ID
% -- update match pool if
a=1;
% calculate distance between each reflector and index 
[match_reflect_vector_pool] = calc_distance(match_reflect_pool,match_reflect_pool_ID);
[match_detected_vector_pool] = calc_distance(match_detected_pool,match_detected_pool_ID);
[Reflect_vec_ID] = index_reflector(match_reflect_vector_pool);
[detected_vec_ID] = index_reflector(match_detected_vector_pool);

% calculate angle between each reflector and index 
[Reflect_angle_vector,Reflect_angle_ID] = calc_angle(match_reflect_pool,match_reflect_pool_ID);
[detect_angle_vector,detect_angle_ID] = calc_angle(match_detected_pool,match_detected_pool_ID);
matched_reflect_ID1=0;
matched_reflect_ID2=0;
matched_detect_ID1=0;
matched_detect_ID2=0;
matched_reflect_vec_ID=0;
matched_reflect_angle_ID=0;
while a==1
    % match distance between two reflectors
    [matched_reflect_ID1,matched_reflect_vec_ID,matched_detect_ID1,matched_detect_vec_ID,dist_result] = match_distance_reflector(match_reflect_vector_pool,Reflect_vec_ID,match_detected_vector_pool,detected_vec_ID,thres_dist_large,thres_dist_match);
    % match angle between three reflectors
    [matched_reflect_ID2,matched_reflect_angle_ID,matched_detect_ID2,matched_detect_angle_ID,angle_result] = match_angle_reflector(Reflect_angle_vector,Reflect_angle_ID,detect_angle_vector,detect_angle_ID,thres_angle_match)
    matched_reflect_ID=matched_reflect_ID1;  %matched_reflect_ID1 for distance match method/ 
    matched_detect_ID=matched_detect_ID1;   %matched_reflect_ID2 for angle match method/ 
    if dist_result==1 && angle_result==1  % check angle_match and distance_match
        match_result=1;
        disp('No matched reflector found, update reference map with new reflectors')
        if num_detect_pool<num_ref_pool && num_detect_pool<length(detected_reflector)
            %% 1.increase the size of matching pool
            num_detect_pool=num_detect_pool+1;
            [match_detected_pool,match_detected_pool_ID] = create_match_detect_pool(num_detect_pool,detected_reflector,detected_ID,Lidar_current_xy);
            %% 2a.calculate the distance with new detected matching pool
            [match_detected_vector_pool] = calc_distance(match_detected_pool,match_detected_pool_ID);
            [detected_vec_ID] = index_reflector(match_detected_vector_pool);
            %% 2b.calculate angle with new matching pool
            [detect_angle_vector,detect_angle_ID] = calc_angle(match_detected_pool,match_detected_pool_ID);           
        elseif num_ref_pool<length(Reflector_map) && num_detect_pool==num_ref_pool
            % increase the size of matching pool
            num_ref_pool=num_ref_pool+1;
            [match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_current_xy);
             % calculate the distance with new reference matching pool           
            [match_reflect_vector_pool] = calc_distance(match_reflect_pool,match_reflect_pool_ID);
            [Reflect_vec_ID] = index_reflector(match_reflect_vector_pool);
            % calculate angle with new matching pool
            [Reflect_angle_vector,Reflect_angle_ID] = calc_angle(match_reflect_pool,match_reflect_pool_ID);
            
        elseif num_ref_pool>=length(Reflector_map) && num_detect_pool>=length(detected_reflector)
            match_result=1;
            disp('Reference matching pool larger than reflector map, No matched distance found!! Mapping stopped')
            break
        else
            break
        end
    elseif dist_result==0 
        disp('Matched distance reflector found!!')
        matched_reflect_ID=matched_reflect_ID1;  %matched_reflect_ID1 for distance match method/ 
        matched_detect_ID=matched_detect_ID1;   %matched_reflect_ID2 for angle match method/ 
        match_result=0;
        break
    elseif angle_result==0
        disp('Matched angled reflector found!!')
        matched_reflect_ID=matched_reflect_ID2;  %matched_reflect_ID1 for distance match method/ 
        matched_detect_ID=matched_detect_ID2;   %matched_reflect_ID2 for angle match method/ 
        match_result=0;
        break
    else
        
    end
end
%% plot map new Lidar scan
%% plot map with random displacement
%plot_reflector(detected_reflector1,detected_ID1,color)
%% 2.c calculate rotation and transition
if match_result == 0
    if length(matched_reflect_ID)==length(matched_detect_ID)
        [ret_R,ret_T,Lidar_update_xy]=locate_reflector_xy(match_reflect_pool,matched_reflect_ID,detected_reflector,matched_detect_ID,Lidar_x,Lidar_y);
        theta_rot=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;  % find the rotation angle from Lidar 
        if abs(theta_rot-rotation_trace(end))>60
            FP= [0 1;-1 0];
            ret_R=ret_R*FP;
            A=match_reflect_pool(matched_reflect_ID,:);
            B=detected_reflector(matched_detect_ID,:);
            centroid_A = mean(A);  %
            centroid_B = mean(B);
            ret_T = -ret_R*centroid_A' + centroid_B';
            theta_rot=-angle(ret_R(1,1)+ret_R(2,1)*i)/pi*180;
        end
        Lidar_update_xy;
        % Calculate reflector rmse errors
        [reflector_rmse]=reflector_rmse_error(ret_R,ret_T,match_reflect_pool,matched_reflect_ID,detected_reflector,matched_detect_ID);
        %% 2.d calculate updated map in the world map
        [Lidar_update_Table,Lidar_update_xy1]=update_Lidar_scan_xy(ret_R,ret_T,measurement_data,scan_data,Lidar_x,Lidar_y);
        % calculate map rmse errors
        [map_rmse]=map_rmse_error(ret_R,ret_T,measurement_data,scan_data);
        %% Plot the reflectors in the world map
        %% Plot update map in the world map
        [detected_ID2,detected_reflector2,reflector_index2]=identify_reflector(amp_thres,dist_thres,reflector_diameter,dist_delta,Lidar_update_Table,scan_data);
        %% Update match reflector pool to get the latest nearest points from reflector map
        [match_reflect_pool,match_reflect_pool_ID] = create_match_ref_pool(num_ref_pool,Reflector_map,Lidar_current_xy);
        %% --Update Lidar trace
        Lidar_trace=[Lidar_trace;Lidar_update_xy];
        rotation_trace=[rotation_trace;theta_rot];
    else
        reflector_rmse=99.99;
        Lidar_trace(end,:)=[0 0];
        rotation_trace=0;
        Lidar_update_Table=0;
        detected_ID2=0;
        detected_reflector2=0;
        match_reflect_pool=0;
        matched_reflect_ID=0;
        
    end
elseif match_result == 1
    disp('Bad data and wait for another scan data');
    reflector_rmse=99.99;   % special value to mark the status
    %Lidar_update_xy=Lidar_trace(end,:);
    Lidar_trace(end,:)=0;
    rotation_trace=0;
    Lidar_update_Table=0;
    detected_ID2=0;
    detected_reflector2=0;
    match_reflect_pool=0;
    matched_reflect_ID=0;
end
end
if reflector_rmse<1
    mea_status=0;
elseif reflector_rmse>1 && reflector_rmse<10
    mea_status=1;
elseif reflector_rmse>10 && reflector_rmse<99.99
    mea_status=2;
elseif reflector_rmse==99.99
    mea_status=3;
else
    mea_status=4;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
end  % Simulation loop end up here!!!

