from setuptools import find_packages, setup

package_name = 'ground_truth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ryan',
    maintainer_email='schofier@oregonstate.edu',
    description='TODO: Package description',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'switch_cntr_test = ground_truth.switch_controller_test:main',
            'pos_test = ground_truth.send_recieve_pos_test:main',
            'joint_test = ground_truth.send_recieve_joint_test:main',
            'pub_tof = ground_truth.publisher_arduino:main',
            'centering = ground_truth.centering:main',
            'centering_cleaned = ground_truth.centering_cleaned:main',
            'centering_cleaned_fake = ground_truth.centering_cleaned_fake:main',
            'centering_cleaned_fake_service = ground_truth.centering_cleaned_fake_services:main',
            'tof1_fake_pub = ground_truth.tof1_fake_pub:main', 
            'tof2_fake_pub = ground_truth.tof2_fake_pub:main', 
            'filter_tof =ground_truth.filter_data:main ', 
            'filter_tof_fake =ground_truth.filter_data_fake:main ', 
            'joystick =ground_truth.joystick:main ', 
            'angle_check_service =ground_truth.check_angle_service:main ', 
            'pub_tool_pose = ground_truth.publisher_tool_pose:main',
            'move_y_service = ground_truth.move_y_direction_service:main',
            'move_y_until_service = ground_truth.move_y_until_service:main',
            'rotate_to_service = ground_truth.rotate_to_service:main',
            'check_angle = ground_truth.check_angle:main',
            'check_angle_fake = ground_truth.check_angle_fake:main',
            'check_angle_no_rot = ground_truth.check_angle_no_rot:main',



            

        ],
    },
)
