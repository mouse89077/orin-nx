from setuptools import setup
import os
import glob

package_name = 'xcorps_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # data_files=[
    #     ('share/ament_index/resource_index/packages',
    #         ['resource/' + package_name]),
    #     ('share/' + package_name, ['package.xml']),
    #     (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    #     (os.path.join('share', package_name, 'config'), glob('xcorps_nav/params/*.yaml'))
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='raspberrypi',
    maintainer_email='raspberrypi@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'heading_calculator = xcorps_nav.heading_calculator:main',
            'gnss_converter = xcorps_nav.gnss_converter:main',
            #'worstcase_vo = xcorps_nav.worstcase_vo',
            'pwm_converter = xcorps_nav.pwm_converter:main',
            'xcorps_gps_rtk_spd = xcorps_nav.xcorps_gps_rtk_spd:main',
            'collision_avoidance = xcorps_nav.collision_avoidance:main',
        ],
    },
)
