#!/bin/bash

# 删除现有数据库
rm -f ~/.ros/rtabmap.db

# 启动RTAB-Map
ros2 launch rtabmap_launch rtabmap.launch.py \
    rate:=20 \
    subscribe_rgb:=true \
    subscribe_depth:=true \
    subscribe_scan:=true \
    rgb_topic:=/camera/camera/color/image_raw \
    depth_topic:=/camera/camera/aligned_depth_to_color/image_raw \
    camera_info_topic:=/camera/camera/color/camera_info \
    imu_topic:=/sensors/imu/imu \
    scan_topic:=/sensors/ladar/scan \
    rgbd_sync:=true \
    subscribe_rgbd:=false \
    compressed:=false \
    approx_sync:=true \
    approx_sync_max_interval:=0.1 \
    loop_closure_enabled:=true \
    loop_closure_reextract:=true \
    loop_closure_time_threshold:=0 \
    loop_closure_max_distance:=3.0 \
    loop_closure_min_inliers:=15 \
    bilateral_filtering:=false \
    queue_size:=20 \
    depth_scale:=1.0 \
    qos:=2 \
    visual_odometry:=false \
    icp_odometry:=true \
    odom_frame_id:=odom \
    frame_id:=base_link \
    map_frame_id:=map \
    publish_tf:=true \
    imu_frame_id:=imu \
    subscribe_odom_info:=true \
    rtabmap_args:="--Vis/MinInliers 15 \
        --Vis/EstimationType 1 \
        --Vis/FeatureType 8 \
        --Vis/MaxFeatures 1200 \
        --Vis/CorNNType 1 \
        --Vis/MinDepth 0.2 \
        --Vis/MaxDepth 12.0 \
        --Vis/Weight 0.0 \
        --Grid/RangeMax 12.0 \
        --Grid/RangeMixMax 12.0 \
        --Grid/FromDepth false \
        --Grid/RayTracing true \
        --Grid/3D false \
        --Grid/2D true \
        --Grid/CellSize 0.02 \
        --Grid/ClusterRadius 0.03 \
        --Grid/DepthMin 0.2 \
        --Grid/DepthMax 12.0 \
        --Grid/GroundIsObstacle true \
        --Grid/FilteringRadius 0.03 \
        --Grid/FilteringAngle 30 \
        --Grid/Sensor true \
        --Grid/NoiseFilteringRadius 0.03 \
        --Grid/NoiseFilteringMinNeighbors 7 \
        --Mem/ImagePreDecimation 2 \
        --Mem/ImagePostDecimation 2 \
        --Mem/IncrementalMemory true \
        --Mem/STMSize 50 \
        --Mem/WorkingMem 150 \
        --Mem/InitWMWithAllNodes true \
        --Mem/NotLinkedNodesKept true \
        --Mem/MaxStMemSize 0 \
        --Mem/RehearsalSimilarity 0.4 \
        --Mem/RecentWmRatio 0.4 \
        --RGBD/LinearUpdate 0.03 \
        --RGBD/AngularUpdate 0.03 \
        --RGBD/OptimizeMaxError 0.8 \
        --RGBD/ProximityMaxGraphDepth 50 \
        --RGBD/ProximityMaxPaths 3 \
        --RGBD/ProximityPathMaxNeighbors 10 \
        --RGBD/ProximityBySpace true \
        --RGBD/ProximityOdomGuess true \
        --RGBD/ProximityPathFilteringRadius 0.2 \
        --RGBD/OptimizeFromGraphEnd false \
        --Optimizer/Strategy 1 \
        --Optimizer/Iterations 150 \
        --Optimizer/Epsilon 0.0005 \
        --Reg/Strategy 2 \
        --Reg/Force3DoF true \
        --Kp/MaxFeatures 0 \
        --Kp/DetectorStrategy 6 \
        --Kp/NNStrategy 1 \
        --Kp/IncrementalDictionary true \
        --Kp/ParallelizeComputation true \
        --Icp/VoxelSize 0.015 \
        --Icp/PointToPlane true \
        --Icp/MaxCorrespondenceDistance 0.05 \
        --Icp/Iterations 50 \
        --Icp/Epsilon 0.0005 \
        --Icp/RangeMin 0.2 \
        --Icp/RangeMax 12.0 \
        --Icp/PointToPlaneGroundNormalsUp 0.9 \
        --Icp/PointToPlaneMinComplexity 0.02 \
        --Icp/DownsamplingStep 1 \
        --Icp/Weight 2.0 \
        --Rtabmap/DetectionRate 15 \
        --Rtabmap/LoopRatio 0.7 \
        --Rtabmap/CreateIntermediateNodes true \
        --RGBD/StartAtOrigin true \
        --Odom/Strategy 1 \
        --Odom/ResetCountdown 0 \
        --Odom/GuessMotion false \
        --Odom/ScanKeyFrameThr 0.5 \
        --Odom/ScanVoxelSize 0.015 \
        --Odom/ScanMaxSize 20000 \
        --Odom/ScanNormalK 15 \
        --Odom/ScanNormalRadius 0.03 \
        --Odom/ScanLocalMap true \
        --Odom/ScanLocalMapDecimation 1 \
        --Odom/ScanMaxRange 12.0 \
        --Odom/ScanMinRange 0.2 \
        --Odom/ScanMinInliers 150 \
        --Odom/ScanIterations 50 \
        --Odom/ScanMaxTranslation 0.3 \
        --Odom/ScanMaxRotation 0.5 \
        --Odom/ScanSubtractRadius 0.05 \
        --Odom/ScanSubtractAngle 0.3 \
        --RGBD/NeighborLinkRefining false \
        --ORB/GpuVersionOpenCL true \
        --params /cloud_enabled=false" \
    args:="--delete-db-on-start"