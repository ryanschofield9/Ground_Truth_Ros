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
            'tof1_fake_pub = ground_truth.tof1_fake_pub:main', 
            'tof2_fake_pub = ground_truth.tof2_fake_pub:main'

        ],
    },
)
