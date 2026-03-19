import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import can
import struct
import time

# --- CONFIGURATION ---
MOTOR_ID = 61        # 0x3D
BITRATE = 500000     # 500k
CHANNEL = 'can0'
INTERFACE = 'socketcan'

# --- TUNING ---
# If the motor spins too fast/slow, adjust this.
ERPM_PER_METER_SECOND = 6000.0 

class BriterMotorNode(Node):

    def __init__(self):
        super().__init__('briter_motor_node')
        
        # 1. Initialize CAN Bus
        try:
            self.bus = can.Bus(interface=INTERFACE, channel=CHANNEL, bitrate=BITRATE)
            self.get_logger().info(f'Connected to CAN: {INTERFACE} @ {BITRATE}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CAN: {e}')
            self.bus = None

        # 2. Initialize State
        self.target_erpm = 0

        # 3. Start Control Timer (20Hz / 0.05s)
        # We run this slower (20Hz) to allow time for both messages to send cleanly.
        self.timer = self.create_timer(0.05, self.control_loop)

        # 4. Create Subscriber
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.get_logger().info('Briter Motor Node Ready. Listening on /cmd_vel...')

    def control_loop(self):
        """
        Sends BOTH Heartbeat and Speed in a strict sequence.
        """
        if self.bus is None:
            return

        try:
            # --- STEP 1: Send Heartbeat (0x00) ---
            # This keeps the motor "Enabled"
            hb_msg = can.Message(arbitration_id=MOTOR_ID, data=[0x00], is_extended_id=False)
            self.bus.send(hb_msg)
            
            # --- CRITICAL: Tiny Sleep ---
            # Wait 2ms to ensure the motor processes the Heartbeat before receiving Speed.
            # This prevents the "Hiccup/Conflict".
            time.sleep(0.002) 

            # --- STEP 2: Send Speed (0x02) ---
            # Pack the Speed: [Instruction 0x02] + [4 Bytes Big Endian Signed Int]
            speed_bytes = list(struct.pack('>i', self.target_erpm))
            data = [0x02] + speed_bytes
            
            speed_msg = can.Message(arbitration_id=MOTOR_ID, data=data, is_extended_id=False)
            self.bus.send(speed_msg)
            
        except can.CanError:
            self.get_logger().error('CAN Bus Error: Failed to send message')

    def listener_callback(self, msg):
        """
        Updates the target speed variable from Teleop input.
        """
        linear_x = msg.linear.x
        self.target_erpm = int(linear_x * ERPM_PER_METER_SECOND)

    def stop_motor(self):
        """
        Safety shutdown
        """
        self.target_erpm = 0
        self.get_logger().info('Stopping motor...')
        
        # Send stop command a few times
        for _ in range(5):
            self.control_loop()
            time.sleep(0.05)
        
        if self.bus:
            self.bus.shutdown()
        self.get_logger().info('CAN bus shutdown complete.')

def main(args=None):
    rclpy.init(args=args)
    node = BriterMotorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_motor()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()