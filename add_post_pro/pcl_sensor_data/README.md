Readme in progress for pcl_sensor_data package

## Nodes
### laser_processor
 - Controls services for starting, stopping, restarting, and processing pointclouds
 - Has a corresponding launch file named "laser_processor.launch"
 - Currently contains the following parameters:
    - start_service_name
        * Name override for service to start recording - default: "/start_recording"
    - stop_service_name
        * Name override for service to pause recording - default: "/stop_recording"
    - restart_service_name
        * Name override for service to resume recording - default: "/restart_recording"
    - process_service_name
        * Name override for service to publish pointcloud compilation of all previous profiles recorded - default :"/process_pointcloud"
    - scanner_frame_id
        * Name of kinematic frame to be used as profile header for pointcloud scans - default: "scancontrol_aperture_frame"

 - To completely begin recording from scratch, call the "start_recording" service. It will wipe the profile buffer. 