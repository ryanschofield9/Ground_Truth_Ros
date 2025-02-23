from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'ground_truth'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
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
            'pub_tof = ground_truth.publisher_arduino:main',
            'centering = ground_truth.centering:main', 
            'filter_tof =ground_truth.filter_data:main ', 
            'joystick =ground_truth.joystick:main ', 
            'check_angle = ground_truth.check_angle:main',
            'touch_tree = ground_truth.touch_tree:main',
            'calc_diameter_service = ground_truth.calc_branch_diameter_service:main',
            'pixel_dimeter= ground_truth.pixel_diameter:main',
            'record_video_service= ground_truth.record_video_service:main',
            'pub_reset= ground_truth.pub_reset:main',
            'Gui= ground_truth.GUI:main',
            'touch= ground_truth.touch:main',
            'pub_camera = ground_truth.camera_pub:main',
            'ROB_541 = ground_truth.Rob_541_proj:main',

        ],
    },
)
