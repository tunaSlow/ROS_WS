import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelUartBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_uart_bridge')
        
        # --- Serial Configuration ---
        # Update '/dev/ttyACM0' to match your STM32's mount point
        self.serial_port_name = '/dev/ttyUSB0'
        self.baud_rate = 115200 
        
        try:
            self.ser = serial.Serial(self.serial_port_name, self.baud_rate, timeout=1)
            self.get_logger().info(f"Successfully connected to STM32 on {self.serial_port_name}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            raise SystemExit

        # --- Subscriber Configuration ---
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Extract linear velocities
        vel_x = msg.linear.x
        vel_y = msg.linear.y
        vel_z = msg.linear.z
        
        # Format the data payload. 
        # Using a start '<' and end '>' marker is a standard embedded practice 
        # to ensure the STM32 can reliably parse the incoming stream.
        payload = f"<{vel_x:.2f},{vel_y:.2f},{vel_z:.2f}>\n"
        
        # Transmit over UART
        try:
            self.ser.write(payload.encode('utf-8'))
            self.get_logger().info(f"Transmitted: {payload.strip()}")
        except serial.SerialException as e:
            self.get_logger().error(f"Serial write error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelUartBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'ser') and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()