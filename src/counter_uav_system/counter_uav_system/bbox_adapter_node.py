#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np

# Possible Inputs
from vision_msgs.msg import Detection3DArray 
from visualization_msgs.msg import MarkerArray, Marker

# Output
from uav_interfaces.msg import PointPillarsData

class BBoxAdapterNode(Node):
    def __init__(self):
        super().__init__('bbox_adapter_node')
        
        # Parameter to switch between standard detections and visualization markers
        self.declare_parameter('use_markers', False)
        self.use_markers = self.get_parameter('use_markers').value
        
        # Parameter for the input topic name
        self.declare_parameter('input_topic', '/pointpillars_bbox')
        input_topic = self.get_parameter('input_topic').value

        self.pub = self.create_publisher(PointPillarsData, '/sensors/point_pillars', 10)

        if self.use_markers:
            # WORKAROUND: Listen to visualization markers if the detection topic is broken
            self.sub = self.create_subscription(
                MarkerArray,
                input_topic,
                self.marker_callback,
                10
            )
            self.get_logger().warn(f"Running in MARKER MODE. Listening for MarkerArray on {input_topic}")
        else:
            # Standard mode
            self.sub = self.create_subscription(
                Detection3DArray,
                input_topic,
                self.detection_callback,
                10
            )
            self.get_logger().info(f"Running in STANDARD MODE. Listening for Detection3DArray on {input_topic}")

    def detection_callback(self, msg):
        """Standard processing for Detection3DArray"""
        if not msg.detections:
            return

        best_detection = None
        max_score = -1.0

        for detection in msg.detections:
            score = detection.results[0].score if detection.results else 0.0
            if score > max_score:
                max_score = score
                best_detection = detection

        if best_detection:
            self.publish_data(
                best_detection.bbox.center.position,
                max_score,
                msg.header
            )

    def marker_callback(self, msg):
        """Fallback processing using Visualization Markers"""
        # Markers are usually for display, but they contain the pose!
        # We look for the first valid CUBE marker (type 1) which usually represents the bbox.
        
        target_marker = None
        
        for marker in msg.markers:
            # Check for ADD action (0) and CUBE type (1)
            # Some implementations uses LINE_STRIP (4) for boxes, but CUBE is standard for solid bbox
            if marker.action == Marker.ADD or marker.action == 0:
                if marker.type == Marker.CUBE: 
                    target_marker = marker
                    break
        
        if target_marker:
            # Markers don't always have a score, so we assume 1.0 or confidence if encoded
            confidence = 1.0 
            self.publish_data(
                target_marker.pose.position,
                confidence,
                target_marker.header
            )

    def publish_data(self, position, confidence, header):
        pf_msg = PointPillarsData()
        pf_msg.header = header
        pf_msg.header.frame_id = "sensor_frame" # Force consistent frame
        
        pf_msg.position.x = float(position.x)
        pf_msg.position.y = float(position.y)
        pf_msg.position.z = float(position.z)
        pf_msg.confidence = float(confidence)
        
        self.pub.publish(pf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BBoxAdapterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()