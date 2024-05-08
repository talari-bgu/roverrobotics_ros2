Issues:
Robot unsual behaviour in rviz2 when using slam.  
reason was topic odometry/filter was different from odometry/wheel  
fix was to turn off the publish_tf from localization_ekf.yaml
