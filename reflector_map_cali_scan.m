%%%--This function do the scan for reflector map calibration
% scan once time and save the scanned reflector map as the reference
% reflector map
function [Reflector_map,Reflector_map_polar,Reflector_ID,status]=reflector_map_cali_scan(amp_thres,dist_thres,reflector_diameter,distance_delta,scan_data,data_source_flag,read_file)

if read_file==1
    fname = ['Lidar_data_example2'];
    scan_data= dlmread( fname, '\t', 3, 0)';
end
Reflector_map=0;
[calibration_data,scan_data]=PolarToRect(Reflector_map,scan_data,data_source_flag);

[detected_ID,detected_reflector,detected_reflector_polar,reflector_index]=identify_reflector(amp_thres,dist_thres,reflector_diameter,distance_delta,calibration_data,scan_data);
detected_reflector
detected_reflector_polar
for ii=1:length(detected_reflector)
    Reflector_ID(ii) = ii;
    Reflector_map(Reflector_ID(ii),1)=detected_reflector(detected_ID(ii),1);   % generate reflector array x
    Reflector_map(Reflector_ID(ii),2)=detected_reflector(detected_ID(ii),2);   % generate reflector array y
    Reflector_map_polar(Reflector_ID(ii),1)=detected_reflector_polar(detected_ID(ii),1);   % generate reflector array x
    Reflector_map_polar(Reflector_ID(ii),2)=detected_reflector_polar(detected_ID(ii),2);   % generate reflector array y
end
status=0;