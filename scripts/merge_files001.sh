# Run this in the folder you want the files to end up in

# rosrun radar_positioning 
datapos=~/Meas_2021-01-08
rosrun radar_positioning aimu_meas_to_bag.py $datapos/MT_stationary_001-000.txt imu_009.bag
#rosrun radar_positioning aimu_meas_to_bag.py $datapos/gps_imu008.txt imu2_008.bag
rosrun radar_positioning acloud_meas_to_bag.py $datapos/RD_stationary_001_UMRR_ID0_Targets_Port066_ID0_20211008114822.csv radar_009.bag

# Merge
rm temp001.bag merged009.bag
#rosrun radar_positioning bagmerge.py -o temp001.bag $datapos/radar_009.bag $datapos/imu_009.bag
#rosrun radar_positioning bagmerge.py -o stationary_009.bag temp001.bag $datapos/imu2_008.bag
rosrun radar_positioning bagmerge.py -o stationary_009.bag $datapos/radar_009.bag $datapos/imu_009.bag
