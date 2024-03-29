import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
from calibration_srv.srv import Calibration
import numpy as np
import cv2
from scipy.spatial.transform import Rotation as R
from cv_bridge import CvBridge
from time import sleep
import copy
# set precision for numpy
np.set_printoptions(precision=3, suppress=True)

SHOW_IMG = False
ARUCO_SIZE = 0.07
ARUCOBASE_X_OFFSET_FROM_BASELINK = 0.0
ARUCOBASE_Y_OFFSET_FROM_BASELINK = - 0.045
ARUCOBASE_Z_OFFSET_FROM_BASELINK = - 0.085
ARUCOBASE_MARKER_ID = 1
ARUCOPUPPET_MARKER_ID = 0
LOCALIZE_PUPPET = True

# Camea offsets

# -90 degree rotation around the x axis
rot_x = np.array([
            [1, 0, 0], 
            [0, 0, 1], 
            [0,-1,0]
            ], dtype=np.float64) 
# # # 90 degree rotation around the z axis
rot_z = np.array([
            [1, 0, 0], 
            [0, 1, 0], 
            [0, 0, 1]
            ], dtype=np.float64)

DEFAULT_ROT = rot_x @ rot_z


# Camera calibration values (ZED2)
# cx=958.084
# cy=528.676
# fx=1077.79
# fy=1078.35
# k1=-0.049069434790845086
# k2=0.0236668632308862
# k3=-0.011162880427934998
# p1=-0.0005055421561811384
# p2=4.10855452151858e-05

# Camera calibration values (RealSense D435)
cx=958.620910644531
cy=564.277893066406
fx=1376.80395507812
fy=1376.80322265625
# real sense has no distortion
k1= 0.0
k2= 0.0
k3= 0.0
p1= 0.0
p2= 0.0

INTRINSIC_CAMERA = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
DISTORTION_CAMERA = np.array([k1, k2, p1, p2, k3])




class PoseService(Node):
    def __init__(self, N):
        super().__init__('pose_service')
        my_callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(Calibration, 'get_camera_pose', self.get_camera_pose_callback, callback_group=my_callback_group)
        self.sub = self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10, callback_group=my_callback_group)
        self.bridge = CvBridge()
        self.received_image = False
        self.N = N
        self.rvec_cam_list = np.zeros((self.N, 3))
        self.tvec_cam_list = np.zeros((self.N, 3))
        self.idx = 0
        # mean is computed just for the aruco base -> the puppet inherits the base frame
        self.mean_tvec = None
        self.mean_quat = None
        self.tvec_puppet = None
        self.quat_puppet = None
        self.computing_avg = False
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.loop_rate = self.create_rate(50, self.get_clock())

    # Function used to estimate the pose of the camera from the ArUco position
    def pose_estimation(self, frame):
        # Convert the image to a grayscale image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Dection of ArUco code
        corners, ids, _ = self.detector.detectMarkers(gray)

        if len(corners) > 0:
            # Avoid crashing if the ArUcos ids are wrong
            if(any(x != 0 and x != 1 for x in ids)):
                return frame,None,None,None,None
            
            ids = ids.reshape(-1)

            # Get Camera position -> camera is needed also for the puppet
            if ARUCOBASE_MARKER_ID in ids:
                cameraIdx = ids.tolist().index(ARUCOBASE_MARKER_ID)

                # Estimate the aruco pose
                rvec_camera, tvec_camera, _ = cv2.aruco.estimatePoseSingleMarkers(corners[cameraIdx], ARUCO_SIZE, INTRINSIC_CAMERA, DISTORTION_CAMERA)
                tvec_puppet = None
                rvec_puppet = None

                if LOCALIZE_PUPPET and ARUCOPUPPET_MARKER_ID in ids:
                    puppetIdx = ids.tolist().index(ARUCOPUPPET_MARKER_ID)
                    rvec_puppet, tvec_puppet, _ = cv2.aruco.estimatePoseSingleMarkers(corners[puppetIdx], ARUCO_SIZE, INTRINSIC_CAMERA, DISTORTION_CAMERA)
                    
                if SHOW_IMG:
                    # Draw the ArUco id on the output image
                    frame = cv2.putText(frame, 'id: '+str(ids[cameraIdx]), (int(corners[cameraIdx][0][0][0]),int(corners[cameraIdx][0][0][1])), cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0), 2, cv2.LINE_AA)

                    # Draw ArUco Corners
                    frame = cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                return  frame, tvec_camera, rvec_camera, tvec_puppet, rvec_puppet
        return frame, None, None, None, None

    # retrieve the camera pose with respect to the base aruco
    def get_camera_pose(self, tvec, quat):
        pose_msg = PoseStamped()
        tvec = tvec.reshape(3, 1)
        rot_matrix = R.from_quat(quat).as_matrix()

        # Apply the default trans
        rot_matrix = rot_matrix @ DEFAULT_ROT

        camera_to_aruco = np.hstack((rot_matrix, tvec))
        camera_to_aruco = np.vstack((camera_to_aruco, [0, 0, 0, 1]))
        aruco_to_camera = np.linalg.inv(camera_to_aruco)
        # create the pose message
        pose_msg.pose.position.x = aruco_to_camera[0, 3] + ARUCOBASE_X_OFFSET_FROM_BASELINK
        pose_msg.pose.position.y = aruco_to_camera[1, 3] + ARUCOBASE_Y_OFFSET_FROM_BASELINK
        pose_msg.pose.position.z = aruco_to_camera[2, 3] + ARUCOBASE_Z_OFFSET_FROM_BASELINK

        # convert the rotation matrix to a quaternion
        r = R.from_matrix(aruco_to_camera[0:3, 0:3])
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = r.as_quat()
        return pose_msg
    
    # retrieve the puppet pose wrt the base aruco
    def get_puppet_pose(self, tvec_base, quat_base, tvec_puppet, quat_puppet):
        pose_msg = PoseStamped()
        tvec_base = tvec_base.reshape(3, 1)
        tvec_puppet = tvec_puppet.reshape(3, 1) 
        rot_matrix_base = R.from_quat(quat_base).as_matrix().reshape(3, 3) @ DEFAULT_ROT
        rot_matrix_puppet = R.from_quat(quat_puppet).as_matrix().reshape(3, 3) @ DEFAULT_ROT

        # Get the relative pose of the puppet wrt the base
        puppet_to_aruco_rot = rot_matrix_base.T @ rot_matrix_puppet
        puppet_to_aruco_pos = rot_matrix_base.T @ (tvec_puppet - tvec_base)

        pose_msg.pose.position.x = puppet_to_aruco_pos[0,0] + ARUCOBASE_X_OFFSET_FROM_BASELINK
        pose_msg.pose.position.y = puppet_to_aruco_pos[1,0] + ARUCOBASE_Y_OFFSET_FROM_BASELINK
        pose_msg.pose.position.z = puppet_to_aruco_pos[2,0] + ARUCOBASE_Z_OFFSET_FROM_BASELINK

        # Apply the default rotation
        r = R.from_matrix(puppet_to_aruco_rot @ DEFAULT_ROT)
        pose_msg.pose.orientation.x, pose_msg.pose.orientation.y, pose_msg.pose.orientation.z, pose_msg.pose.orientation.w = r.as_quat()

        return pose_msg
    def quaternion_avg(self, quat_list):        
        # average
        quat_list = np.array(quat_list)
        M = np.zeros((4, 4))
        for i in range(quat_list.shape[0]):
            M = M + 1/self.N * quat_list[i].T @ quat_list[i]
        
        # extract the eigenvector corresponding to the largest eigenvalue
        w, v = np.linalg.eig(M)
        return v[:, np.argmax(w)]

    def image_callback(self, msg):
        if self.computing_avg:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            image = copy.deepcopy(self.image)
            frame, tvec_camera, rvec_camera, self.tvec_puppet, rvec_puppet = self.pose_estimation(image)
            if SHOW_IMG:
                cv2.imshow('Image', frame)
                cv2.waitKey(1)
            if rvec_camera is None:
                self.get_logger().error('No Aruco found')
            else:
                self.tvec_cam_list[self.idx] = tvec_camera[0][0]
                self.rvec_cam_list[self.idx] = rvec_camera[0][0]
                self.get_logger().info('Collecting position number: ' + str(self.idx))
                self.idx += 1

            if self.idx == self.N:
                if (rvec_puppet is not None and self.tvec_puppet is not None) and LOCALIZE_PUPPET:
                    self.quat_puppet = R.from_matrix(cv2.Rodrigues(rvec_puppet)[0]).as_quat().reshape(1, 4)
                self.computing_avg = False
                self.get_logger().info('Computing mean')
                self.mean_tvec = np.mean(self.tvec_cam_list, axis=0)
                # convert from rodriquez to quaternion
                quat_list = [] 
                for i in range(len(self.rvec_cam_list)):
                    rot_mat = cv2.Rodrigues(self.rvec_cam_list[i])[0]
                    quat_list.append(R.from_matrix(rot_mat).as_quat().reshape(1, 4))
                # self.mean_quat = self.quaternion_avg(quat_list)
                # self.mean_quat = np.mean(self.rvec_cam_list, axis=0)
                self.mean_quat = self.quaternion_avg(quat_list)
                
                self.tvec_cam_list = np.zeros((self.N, 3))
                self.rvec_cam_list = np.zeros((self.N, 3))
                self.idx = 0


    def get_camera_pose_callback(self, request, response):
        self.get_logger().info('Request received')
        response.camera_pose = PoseStamped()
        response.puppet_pose = PoseStamped()

        i=0
        self.computing_avg = True
        # wait two seconds until the mean is computed
        while  rclpy.ok() and self.mean_tvec is None and i<200: # 2 seconds
            # rate.sleep()
            i+=1
            self.loop_rate.sleep()

        if(i==200 and self.mean_tvec is None):
            self.get_logger().error('No Aruco found!!')
            response.camera_pose.header = Header()
            response.camera_pose.header.frame_id = "error"
            return response

        camera_pose_msg = self.get_camera_pose(self.mean_tvec, self.mean_quat)

        # Populate ROS message
        camera_pose_msg.header = Header()
        camera_pose_msg.header.frame_id = "base_link"
        camera_pose_msg.header.stamp = self.get_clock().now().to_msg()
        response.camera_pose = camera_pose_msg

        if LOCALIZE_PUPPET and self.tvec_puppet is not None and self.quat_puppet is not None:
            puppet_pose_msg = self.get_puppet_pose(self.mean_tvec, self.mean_quat, self.tvec_puppet, self.quat_puppet)
            puppet_pose_msg.header = Header()
            puppet_pose_msg.header.frame_id = "base_link"
            puppet_pose_msg.header.stamp = self.get_clock().now().to_msg()
            response.puppet_pose = puppet_pose_msg
            self.tvec_puppet = None
            self.quat_puppet = None
        else:
            response.puppet_pose.header = Header()
            response.puppet_pose.header.frame_id = "error"
            self.get_logger().error('No puppet found!')

        self.mean_tvec = None
        self.mean_quat = None

        self.get_logger().info('Pose sent')
        return response

def main(args=None):
    rclpy.init(args=args)
    pose_service = PoseService(N=10)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(pose_service)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
