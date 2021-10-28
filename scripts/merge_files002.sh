# Run this in the folder you want the files to end up in

# rosrun radar_positioning 
datapos=~/Meas_2021-10-26
rosrun radar_positioning aimu_meas_to_bag.py $datapos/imu-L01.txt imu_016.bag
#rosrun radar_positioning aimu_meas_to_bag.py $datapos/gps_imu008.txt imu2_008.bag
rosrun radar_positioning acloud_meas_to_bag.py $datapos/radar_L01.csv radar_016.bag

# Merge
#rm straight012.bag
#rosrun radar_positioning bagmerge.py -o temp001.bag $datapos/radar_009.bag $datapos/imu_009.bag
#rosrun radar_positioning bagmerge.py -o stationary_009.bag temp001.bag $datapos/imu2_008.bag
rosrun radar_positioning bagmerge.py -o lshape_016.bag $datapos/radar_016.bag $datapos/imu_016.bag
