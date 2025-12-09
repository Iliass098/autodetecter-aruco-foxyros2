import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')

        self.get_logger().info("Aruco detector running (Gazebo mode)")

        # Diccionario correcto para tu tablero
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Suscripción a la cámara de Gazebo
        self.subscription = self.create_subscription(
            Image,
            '/cam/image_raw',
            self.image_callback,
            10
        )

        # Tamaño REAL del marcador en metros
        self.marker_size = 0.05  # 5 cm (cámbialo si es otro tamaño)

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, _ = aruco.detectMarkers(
            gray, 
            self.aruco_dict, 
            parameters=self.parameters
        )

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            self.get_logger().info(f"Detected markers: {ids.flatten()}")

            for i, marker_id in enumerate(ids):
                self.publish_tf(marker_id[0], corners[i], frame)

        cv2.imshow("Gazebo ArUco Detection", frame)
        cv2.waitKey(1)

    def publish_tf(self, marker_id, corners, frame):
        # Cámara en FOXY → necesitas la intrínseca
        # Para empezar, ponemos una intrínseca estándar (ajústala si tienes la tuya)
        fx = 554.0
        fy = 554.0
        cx = frame.shape[1] / 2
        cy = frame.shape[0] / 2

        camera_matrix = np.array([[fx, 0, cx],
                                  [0, fy, cy],
                                  [0,  0,  1]])

        dist_coeffs = np.zeros((5, 1))  # sin distorsión para Gazebo

        # solvePnP → posición 3D del marcador
        ret = cv2.solvePnP(
            np.array([
                [-self.marker_size/2,  self.marker_size/2, 0],  # top-left
                [ self.marker_size/2,  self.marker_size/2, 0],  # top-right
                [ self.marker_size/2, -self.marker_size/2, 0],  # bottom-right
                [-self.marker_size/2, -self.marker_size/2, 0]   # bottom-left
            ]),
            corners.reshape(4, 2),
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if not ret[0]:
            self.get_logger().warn("PNP failed")
            return

        rvec, tvec = ret[1], ret[2]

        # Convert rotation vector to quaternion
        R, _ = cv2.Rodrigues(rvec)
        quat = self.rotation_matrix_to_quaternion(R)

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"aruco_{marker_id}"

        t.transform.translation.x = float(tvec[0])
        t.transform.translation.y = float(tvec[1])
        t.transform.translation.z = float(tvec[2])

        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

    def rotation_matrix_to_quaternion(self, R):
        q = np.empty(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2,1] - R[1,2]) * s
            q[1] = (R[0,2] - R[2,0]) * s
            q[2] = (R[1,0] - R[0,1]) * s
        else:
            if R[0,0] > R[1,1] and R[0,0] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[0,0] - R[1,1] - R[2,2])
                q[3] = (R[2,1] - R[1,2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0,1] + R[1,0]) / s
                q[2] = (R[0,2] + R[2,0]) / s
            elif R[1,1] > R[2,2]:
                s = 2.0 * np.sqrt(1.0 + R[1,1] - R[0,0] - R[2,2])
                q[3] = (R[0,2] - R[2,0]) / s
                q[0] = (R[0,1] + R[1,0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1,2] + R[2,1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2,2] - R[0,0] - R[1,1])
                q[3] = (R[1,0] - R[0,1]) / s
                q[0] = (R[0,2] + R[2,0]) / s
                q[1] = (R[1,2] + R[2,1]) / s
                q[2] = 0.25 * s
        return q

def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

