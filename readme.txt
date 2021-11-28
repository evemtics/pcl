首先需要通过
catkin_create_pkg pcl pcl_conversions pcl_ros roscpp sensor_msgs rospy
创建一个pcl的包
随后catkin_make编译
其中：
1.pcl_create.cpp文件中创建一个随机点云并通过ros和pcl之间的消息转换机制，将消息发送出去，运行节点为rosrun pcl pcl_create
2.pcl_read.cpp 读取一个pcd点云文件并将消息发送出去，rosrun pcl pcl_read
3.pcl_write.cpp 订阅前面两个节点之一，并接收发送出的点云消息存储到点云文件中，rosrun pcl pcl_write
4. pcl_filter.cpp 读取一个点云文件，进行统计滤波，并可视化点云，rosrun pcl pcl_filter
5. pcl_Radius.cpp 进行半径滤波，可视化点云 rosrun pcl pcl_radius
6. pcl_Gassian.cpp 高斯滤波，可视化点云 rosrun pcl pcl_radius
7. pcl_downSample.cpp 体素栅格滤波器，可视化点云 rosrun pcl pcl_downSample
8.sansac_line.cpp 随机抽样一致性拟合直线
