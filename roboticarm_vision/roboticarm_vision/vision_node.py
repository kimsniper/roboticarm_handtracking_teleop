import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from std_msgs.msg import Float64

import cv2
import mediapipe as mp
import numpy as np


class VisionNode(Node):

    def __init__(self):
        super().__init__('vision_node')

        self.pos_pub = self.create_publisher(Point, '/hand_tracking/position', 10)
        self.pinch_pub = self.create_publisher(Float64, '/hand_tracking/pinch', 10)

        self.cap = cv2.VideoCapture(0)

        # MediaPipe
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=1,
            model_complexity=1,
            min_detection_confidence=0.7,
            min_tracking_confidence=0.7
        )

        self.mp_draw = mp.solutions.drawing_utils

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_z = 0.0

        self.alpha = 0.25  # smoothing factor

        self.timer = self.create_timer(0.03, self.loop)

        self.get_logger().info("Vision node started")

    def smooth(self, prev, new):
        return self.alpha * new + (1.0 - self.alpha) * prev

    def loop(self):

        ret, frame = self.cap.read()
        if not ret:
            return

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(rgb)

        if result.multi_hand_landmarks:

            hand = result.multi_hand_landmarks[0]

            # Key landmarks
            wrist = hand.landmark[0]
            index_tip = hand.landmark[8]
            thumb_tip = hand.landmark[4]
            middle_mcp = hand.landmark[9]

            # Normalize position (0–1)
            x = wrist.x
            y = wrist.y

            # Depth estimation based on hand size in image
            hand_depth = np.linalg.norm([
                wrist.x - middle_mcp.x,
                wrist.y - middle_mcp.y
            ])

            z = hand_depth

            # Pinch distance for gripper control
            pinch = np.linalg.norm([
                thumb_tip.x - index_tip.x,
                thumb_tip.y - index_tip.y
            ])

            # Smooth signals
            x = self.smooth(self.prev_x, x)
            y = self.smooth(self.prev_y, y)
            z = self.smooth(self.prev_z, z)

            self.prev_x = x
            self.prev_y = y
            self.prev_z = z

            pos_msg = Point()
            pos_msg.x = x
            pos_msg.y = y
            pos_msg.z = z

            self.pos_pub.publish(pos_msg)

            pinch_msg = Float64()

            # Convert pinch to gripper scale
            pinch_msg.data = pinch

            self.pinch_pub.publish(pinch_msg)

            self.mp_draw.draw_landmarks(
                frame,
                hand,
                self.mp_hands.HAND_CONNECTIONS
            )

            cv2.circle(frame, (int(w * x), int(h * y)), 10, (0, 255, 0), -1)

        cv2.imshow("Hand Tracking", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()