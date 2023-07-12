from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()
    
    GPS_node = Node(
        package='xcorps_nav',
        executable='xcorps_gps_rtk_spd',
        name= 'GPS_Publisher',
        namespace='OS' 
    )
    ld.add_action(GPS_node)
    
    IMU_node = Node(
        package='bno055',
        executable='bno055',
        name= 'IMU_Publisher',
        namespace='OS' 
    )
    ld.add_action(IMU_node)
    
    GNSS_Converter_node = Node(
        package='xcorps_nav',
        executable='gnss_converter',
        name= 'GNSS_Converter',
        namespace='OS' 
    )
    ld.add_action(GNSS_Converter_node)
    
    Heading_Calculator_node = Node(
        package='xcorps_nav',
        executable='heading_calculator',
        name= 'Heading_Calculator',
        namespace='OS' 
    )
    ld.add_action(Heading_Calculator_node)
    
    Collision_Avoidance_node = Node(
        package='xcorps_nav',
        executable='collision_avoidance',
        name= 'Collision_Avoidance',
        namespace='OS' 
    )
    ld.add_action(Collision_Avoidance_node)
    
    PWM_Converter_node = Node(
        package='xcorps_nav',
        executable='pwm_converter',
        name= 'PWM_Converter',
        namespace='OS' 
    )
    ld.add_action(PWM_Converter_node)
    
    return ld
    
    
    
    
    
    
