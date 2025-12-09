import rclpy
from rclpy.node import Node
import cv2
import os
import cv2.aruco as aruco

class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.get_logger().info('ArUco detector node started')

        # Folder parameter
        self.declare_parameter('image_folder', '/home/ilias/aruco./aruco_localization/images')
        self.image_folder = self.get_parameter('image_folder').value

        if os.path.exists(self.image_folder) and len(os.listdir(self.image_folder)) > 0:
            self.get_logger().info(f'Processing images from: {self.image_folder}')
            self.process_images()
        else:
            self.get_logger().info('No images found, opening camera...')
            self.process_camera()

    def process_images(self):
        imgs = [f for f in os.listdir(self.image_folder) if f.endswith(('.png', '.jpg', '.jpeg'))]

        for img_name in imgs:
            path = os.path.join(self.image_folder, img_name)
            img = cv2.imread(path)

            if img is None:
                self.get_logger().warn(f'Cannot read image: {img_name}')
                continue

            self.detect_and_show(img, img_name)

    def process_camera(self):
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            self.get_logger().error('Cannot open camera')
            return

        self.get_logger().info("Press Q to quit camera")
        while rclpy.ok():
            ret, frame = cap.read()
            if not ret:
                break

            self.detect_and_show(frame, "Camera")

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()

    def detect_and_show(self, img, win_name):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # IMPORTANT — the correct dictionary for your map image!
        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        parameters = aruco.DetectorParameters()

        # More robust detection on printed/drawn markers
        parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

        if ids is not None:
            ids = ids.flatten()
            self.get_logger().info(f"Detected markers ({len(ids)}): {ids}")

            for i, corners_i in enumerate(corners):
                marker_id = ids[i]

                # Compute marker center
                c = corners_i[0]
                cx = int((c[0][0] + c[1][0] + c[2][0] + c[3][0]) / 4)
                cy = int((c[0][1] + c[1][1] + c[2][1] + c[3][1]) / 4)

                self.get_logger().info(f"Marker ID {marker_id} at pixel position ({cx}, {cy})")

            aruco.drawDetectedMarkers(img, corners, ids)
        else:
            self.get_logger().info(f"No markers detected in {win_name}")

        cv2.imshow(f"Detection - {win_name}", img)
        if win_name != "Camera":
            cv2.waitKey(0)
            cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

