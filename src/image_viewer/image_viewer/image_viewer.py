import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image

from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State

import cv2
import numpy as np


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_viewer')
        self.declare_parameter('vehicle_id', 0)
        self.declare_parameter('vehicle_type', 'x500')
        self.declare_parameter('camera_type', 'front')

        self.vehicle_id = self.get_parameter('vehicle_id').get_parameter_value().integer_value
        self.vehicle_type = self.get_parameter('vehicle_type').get_parameter_value().string_value
        self.camera_type = self.get_parameter('camera_type').get_parameter_value().string_value

        self.get_logger().info(
            f"Configure ImageViewer {self.vehicle_type}_{self.vehicle_id} camera={self.camera_type}"
        )

        receiver_node_name = (
            f'image_receiver_{self.vehicle_type}_{self.vehicle_id}_{self.camera_type}'
        )
        self.cli = self.create_client(ChangeState, f'/{receiver_node_name}/change_state')
        if not self.cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available after waiting')
            raise RuntimeError('Service not available')

        self.req = ChangeState.Request()

        # Query current lifecycle state and perform only necessary transitions
        get_state_cli = self.create_client(GetState, f'/{receiver_node_name}/get_state')
        get_state_cli.wait_for_service(timeout_sec=5.0)
        future = get_state_cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, future)
        current_state = future.result().current_state.id if future.result() else State.PRIMARY_STATE_UNKNOWN

        self.get_logger().info(f'image_receiver current state id: {current_state}')

        if current_state == State.PRIMARY_STATE_UNCONFIGURED:
            self.send_request(Transition.TRANSITION_CONFIGURE)
            self.send_request(Transition.TRANSITION_ACTIVATE)
        elif current_state == State.PRIMARY_STATE_INACTIVE:
            self.send_request(Transition.TRANSITION_ACTIVATE)
        elif current_state == State.PRIMARY_STATE_ACTIVE:
            self.get_logger().info('image_receiver already active, skipping lifecycle setup')

        # ROS namespaces are 1-based, unlike the 0-based vehicle_id:
        # /vehicle1 belongs to vehicle_id 0.
        topic = (
            f'/vehicle{self.vehicle_id + 1}'
            f'/camera/{self.camera_type}/image_raw'
        )
        self.subscription = self.create_subscription(
            Image,
            topic,
            self.listener_callback,
            qos_profile_sensor_data,
        )

    def listener_callback(self, msg):
        try:
            dtype = np.uint8
            encoding = msg.encoding
            if encoding in ('rgb8', 'bgr8'):
                channels = 3
            elif encoding in ('rgba8', 'bgra8'):
                channels = 4
            elif encoding == 'mono8':
                channels = 1
            else:
                channels = 3

            frame = np.frombuffer(msg.data, dtype=dtype).reshape(
                (msg.height, msg.width, channels)
            )

            if encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            elif encoding == 'rgba8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            elif encoding == 'bgra8':
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            cv2.imshow(
                f"vehicle{self.vehicle_id + 1}/{self.camera_type}",
                frame,
            )
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Failed to display image: {e}")

    def send_request(self, transition_id, timeout_sec=None):
        self.req.transition.id = transition_id
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future, timeout_sec=timeout_sec)
        return self.future.result()

    def restore_lifecycle(self):
        """Stream lifetime == viewer lifetime: on exit (Ctrl-C / window
        close) the receiver is always driven back to UNCONFIGURED,
        regardless of who activated it. Bounded waits: shutdown must never
        hang on a dead receiver."""
        for transition in (Transition.TRANSITION_DEACTIVATE,
                           Transition.TRANSITION_CLEANUP):
            try:
                self.send_request(transition, timeout_sec=3.0)
            except Exception as e:
                self.get_logger().warning(f'lifecycle teardown failed: {e}')
        self.get_logger().info('receiver stream stopped')


def main(args=None):
    # Keep SIGINT as a plain python KeyboardInterrupt: rclpy's own handler
    # would tear the context down BEFORE our finally runs, making the
    # lifecycle-restore service calls impossible.
    rclpy.init(args=args,
               signal_handler_options=rclpy.signals.SignalHandlerOptions.NO)
    image_subscriber = ImageSubscriber()
    try:
        # spin_once with a timeout instead of a bare spin(): with rclpy's
        # signal handlers disabled, a C-blocked spin defers KeyboardInterrupt
        # until a message arrives - which may be never while no stream flows.
        while rclpy.ok():
            rclpy.spin_once(image_subscriber, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.restore_lifecycle()
        image_subscriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
