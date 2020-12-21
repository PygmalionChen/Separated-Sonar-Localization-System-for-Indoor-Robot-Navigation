此工程测试使用仿真系统输出lidar真值,与超声波数据优化反解sonar位姿.
超声波数据以 n= 60进行分割,数据时序对应50ms仿真周期,除了POSE获取不通过NDT,剩下的优化遵循原始lidar_sonar工程.
每一帧sonar数据对应一帧pose,pose以 n = 2 分割.

