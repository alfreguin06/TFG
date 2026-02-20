import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench, Vector3
from rcl_interfaces.msg import ParameterDescriptor
from PyKDL import Rotation, Vector
from .Jr3Manager import JR3Manager

class Jr3Driver(Node):

    def __init__(self):
        super().__init__('jr3_driver')

        channel_param = self.declare_parameter('channel', 'COM3',
            ParameterDescriptor(description='Serial channel name'))
        baudrate_param = self.declare_parameter('baudrate', 115200,
            ParameterDescriptor(description='Serial baudrate (bps)'))
        cutoff_param = self.declare_parameter('cutoff_frecuency', 200,
            ParameterDescriptor(description='Cutoff frequency (Hz)'))
        period_param = self.declare_parameter('acquisition_period', 10000,
            ParameterDescriptor(description='Acquisition period (s)'))
        publish_rate_param = self.declare_parameter('publish_rate', 0.1,
            ParameterDescriptor(description='Publish rate (s)'))
        jr3_roll_param = self.declare_parameter('jr3_roll', 0.0,
            ParameterDescriptor(description='JR3 frame roll (rad)'))
        jr3_pitch_param = self.declare_parameter('jr3_pitch', 0.0,
            ParameterDescriptor(description='JR3 frame pitch (rad)'))
        jr3_yaw_param = self.declare_parameter('jr3_yaw', 0.0,
            ParameterDescriptor(description='JR3 frame yaw (rad)'))
        jr3_deadband_forces_param = self.declare_parameter('jr3_deadband_forces', 0.0,
            ParameterDescriptor(description='JR3 deadband on force measurements (N)'))
        jr3_deadband_torques_param = self.declare_parameter('jr3_deadband_torques', 0.0,
            ParameterDescriptor(description='JR3 deadband on torque measurements (N*m)'))

        self.R_jr3_tcp = Rotation.RPY(jr3_roll_param.get_parameter_value().double_value,
                                      jr3_pitch_param.get_parameter_value().double_value,
                                      jr3_yaw_param.get_parameter_value().double_value)

        self.jr3_deadband_forces = jr3_deadband_forces_param.get_parameter_value().double_value
        self.jr3_deadband_torques = jr3_deadband_torques_param.get_parameter_value().double_value

        self.publisher = self.create_publisher(Wrench, 'jr3', 10)
        return
        
        self.jr3 = JR3Manager(channel=channel_param.get_parameter_value().string_value,
                              baudrate=baudrate_param.get_parameter_value().integer_value)
        
        self.jr3.start(fc=cutoff_param.get_parameter_value().integer_value,
                       period=period_param.get_parameter_value().integer_value)
        
        time.sleep(1)
        jr3.zero_offs()

        self.timer = self.create_timer(publish_rate_param.get_parameter_value().float_value,
                                       self.timer_callback)

    def timer_callback(self):
        success, forces, torques = self.jr3.read()

        if success:
            kdl_forces = Vector(forces[0], forces[1], forces[2])
            kdl_torques = Vector(torques[0], torques[1], torques[2])

            if kdl_forces.Norm() < self.jr3_deadband_forces:
                kdl_forces = Vector.Zero()
            
            if kdl_torques.Norm() < self.jr3_deadband_torques:
                kdl_torques = Vector.Zero()

            msg.force = self.R_jr3_tcp * kdl_forces
            msg.torque = self.R_jr3_tcp * kdl_torques

            self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    jr3_driver = Jr3Driver()
    rclpy.spin(jr3_driver)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
