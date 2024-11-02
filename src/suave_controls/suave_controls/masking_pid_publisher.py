import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from geometry_msgs.msg import Vector3
import os
import cv2
import numpy as np
from simple_pid import PID  # type: ignore
import csv
import time

def apply_mask(frame):
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define range of neon yellow color in HSV
    lower_pink = np.array([140, 50, 50])  # Lower bound for pink
    upper_pink = np.array([170, 255, 255])  # Upper bound for pink

    # Create masks. Threshold the HSV image to get only pink colors
    mask = cv2.inRange(hsv, lower_pink, upper_pink)

    # Bitwise-AND mask and original image
    result = cv2.bitwise_and(frame, frame, mask=mask)
    return result, mask

def find_and_draw_contours(frame, mask):
    # Find contours in the mask
    contours = None
    contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # If no contours are found, return the original frame
    if not contours:
        return frame, None, None, None

    # Find the largest contour by area
    largest_contour = max(contours, key=cv2.contourArea)

    # Get the bounding box coordinates of the largest contour
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

    # Calculate the center coordinates of the bounding box
    center_x_box = x + w // 2
    center_y_box = y + h // 2

    # Draw the center point of the bounding box
    center_point = (int(center_x_box), int(center_y_box))
    cv2.circle(frame, center_point, 5, (255, 0, 0), -1)  # Blue circle for center point

    # Estimate depth (for demonstration, replace with actual depth calculation if available)
    depth = w  # Example: using width as a proxy for depth

    return frame, center_x_box, center_y_box, depth

#         P, I, D
XGAINS = [1, 0, 0]
YGAINS = [1, 0, 0]
ZGAINS = [1, 0, 0]

class MaskingPIDPublisher(Node):

    def __init__(self):
        super().__init__('masking_pid_publisher')
        self.publisher_ = self.create_publisher(Vector3, 'masking_pid_publisher', 10)
        
        self.bounding_box_data = []

        self.frame_width = 640  # Adjust to your camera frame width
        self.frame_height = 480  # Adjust to your camera frame height
        self.center_x = self.frame_width / 2
        self.center_y = self.frame_height / 2
        self.setpoint_depth = 16  # Example setpoint for depth in pixels

        self.start_time = time.time()

        self.pid_x = PID(XGAINS[0], XGAINS[1], XGAINS[2], setpoint=self.center_x)  # PID for horizontal position
        self.pid_y = PID(YGAINS[0], YGAINS[1], YGAINS[2], setpoint=self.center_y)  # PID for vertical position
        self.pid_depth = PID(ZGAINS[0], ZGAINS[1], ZGAINS[2], setpoint=self.setpoint_depth)  # PID for depth position

        self.pid_x.output_limits = (-100, 100)  # Adjust as needed for your application
        self.pid_y.output_limits = (-100, 100)  # Adjust as needed for your application
        self.pid_depth.output_limits = (-100, 100)  # Adjust as needed for your application

        self._srv_export = self.create_service(Empty, 'export', self.service_export)


    def livestream_from_camera(self, camera_index=0):
        # Open a connection to the camera
        contours = None
        cap = cv2.VideoCapture(camera_index)

        if not cap.isOpened():
            print("Error: Could not open camera.")
            return

        while True:
            # Capture frame-by-frame
            ret, frame = cap.read()

            if not ret:
                print("Error: Failed to capture image")
                break

            # Apply mask to the frame
            masked_frame, mask = apply_mask(frame)

            # Find and draw contours on the masked frame
            frame_with_contours, center_x_box, center_y_box, depth = find_and_draw_contours(masked_frame, mask)

            if center_x_box is not None and center_y_box is not None:
                contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                x, y, w, h = cv2.boundingRect(largest_contour)  # Get bounding box dimensions

                # Compute the control outputs using PID
                control_x = 0
                if (center_x_box is not None):
                    control_x = self.pid_x(center_x_box)
                
                control_y = 0
                if (center_y_box is not None):
                    control_y = self.pid_y(center_y_box)

                control_depth = 0
                if (depth is not None):
                    control_depth = self.pid_depth(depth)

                # Output the center x, center y, depth, and control values to the terminal
                try:
                    print(f'Center X: {center_x_box:.2f}\tControl X: {control_x:.2f}')
                    print(f'Center Y: {center_y_box:.2f}\tControl Y: {control_y:.2f}')
                    print(f'Depth: {depth:.2f}\t\tControl Depth: {control_depth:.2f}')
                except:
                    print("Something went wrong printing...")

                msg = Vector3()
                msg.x = float(control_x)
                msg.y = float(control_y)
                msg.z = float(control_depth)

                self.publisher_.publish(msg)
                self.get_logger().info('Publishing masking pid publisher!')
                rclpy.spin_once(self, timeout_sec=0)

                dt = time.time() - self.start_time
                bounding_box_info = {
                    'time': dt,
                    'width': w,  
                    'height': h,  
                    'center_x': center_x_box,
                    'center_y': center_y_box,
                    'pid_x': control_x,
                    'pid_y': control_y,
                    'pid_depth': control_depth
                }
                self.bounding_box_data.append(bounding_box_info)

                self.get_logger().info(f'Bounding box data collected: {bounding_box_info}')


            # Display the resulting frame and mask
            cv2.imshow('Live Stream with Bounding Box and Center Point', frame_with_contours)
            cv2.imshow('Mask', mask)

            # Press 'q' to quit the livestream
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        # When everything done, release the capture
        cap.release()
        cv2.destroyAllWindows()

    def service_export(self, request, response):
        self.get_logger().info('Export service called.')

        # TODO: Lets export this generically, maybe a file like ~/Data/MaskingPidData/MM_dd_HH_mm_ss.csv

        csv_filename = time.strftime("%m_%d_%H_%M_%S.csv")
        csv_file_path = '/home/suave/Data/SuaveMaskingPid/%s' % csv_filename
        file_exists = os.path.isfile(csv_file_path)
        
        with open(csv_file_path, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file_exists:
                writer.writerow(['time', 'Width', 'Height', 'Center X', 'Center Y', 'PID X', 'PID Y', 'PID Depth'])
            for data in self.bounding_box_data:
                writer.writerow([
                    data['time'],
                    data['width'],
                    data['height'],
                    data['center_x'],
                    data['center_y'],
                    data['pid_x'],
                    data['pid_y'],
                    data['pid_depth']
                ])  
        self.get_logger().info('Exported bounding box data to CSV file!')
        self.bounding_box_data.clear()
        os.system("cp %s /home/suave/Data/SuaveMaskingPid/latest.csv" % csv_file_path)
        return response
    
def test_export(maskingPIDPublisher):
    maskingPIDPublisher.bounding_box_data.append({
        'width': 100,
    })
    maskingPIDPublisher.service_export(None, None)

def main(args=None):
    print("[main] suave_controls/masking_pid_publisher.py")

    rclpy.init(args=args)

    maskingPIDPublisher = MaskingPIDPublisher()

    maskingPIDPublisher.livestream_from_camera()

    maskingPIDPublisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
