import numpy as np

# Camera calibration values (RealSense D435) - 848x480
cx_RealSense = 421.9996032714844
cy_RealSense = 237.9039764404297
fx_RealSense = 616.7684936523438
fy_RealSense = 615.880615234375
k1_RealSense = 0.0
k2_RealSense = 0.0
k3_RealSense = 0.0
p1_RealSense = 0.0
p2_RealSense = 0.0

INTRINSIC_CAMERA_REALSENSE = np.array([[fx_RealSense, 0, cx_RealSense], [0, fy_RealSense, cy_RealSense], [0, 0, 1]])
DISTORTION_CAMERA_REALSENSE = np.array([k1_RealSense, k2_RealSense, p1_RealSense, p2_RealSense, k3_RealSense])

# Camera calibration values (Sony 8MegaPixel) - 1280x960
cx_RealSense_1280 = 642.2582577578172
cy_RealSense_1280 = 474.1471906434584
fx_RealSense_1280 = 999.461170663331
fy_RealSense_1280 = 996.9611451866272
k1_RealSense_1280 = 0.164427473610091
k2_RealSense_1280 = -0.2717244716038656
k3_RealSense_1280 = 0.0
p1_RealSense_1280 = -0.002867946281892625
p2_RealSense_1280 = -9.69782173585606e-05

# Position torso camera respect to odom frame
position_odom_camera = [-0.106549417688, 0.0214074875957, 1.09765735695]
orientation_odom_camera = [0.00679285729054, 0.36165720942, -0.0031636557936, 0.932281010128]

# [w,x,y,z]
baseFootprint_to_camera = { "position" : [-0.096188350602, 0.0323055329919, 1.09765735695] , "orientation": [0.93228139161, 0.00674844400512, 0.361658040888, -0.00304916720499]}