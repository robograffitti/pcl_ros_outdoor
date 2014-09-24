#!/bin/bash
# $1=file.pcd, $2=/frame
:<<'#__COMMENT_OUT__'
echo "$# arguments."
if [ $# -eq 2 ]; then
    rosrun pcl_ros pcd_to_pointcloud $1 _frame_id:=$2 $3
elif [ $# -eq 2 ]; then # for improving readability
    rosrun pcl_ros pcd_to_pointcloud $1 _frame_id:=$2
elif [ $# -eq 1 ]; then # Conditional Eq: [ num1 -option num2 ]
    rosrun pcl_ros pcd_to_pointcloud $1 _frame_id:=/odom
elif [ $# -eq 0 ]; then
    echo "You need 3 arguments."
else # 3 or more arguments
    echo "Only first 2 arguments are evaluated."
    rosrun pcl_ros pcd_to_pointcloud $1 _frame_id:=$2
fi

#exit 0
#__COMMENT_OUT__

PCD_FILE="tomato_si.pcd"
FRAME_ID="/odom"
INTERVAL=1.000

# rosrun pcl_ros pcd_to_pointcloud <file.pcd> [ <interval> ]
rosrun pcl_ros pcd_to_pointcloud $PCD_FILE _frame_id:=$FRAME_ID $INTERVAL