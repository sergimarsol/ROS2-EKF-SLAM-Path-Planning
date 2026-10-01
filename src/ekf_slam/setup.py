from setuptools import find_packages, setup

package_name = 'ekf_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
        ('share/' + package_name + '/launch', ['launch/ekf_slam_node.launch.py']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'scipy',
        'pyyaml'
    ],
    zip_safe=True,
    maintainer='Sergi Marsol, Yule Zhang',
    maintainer_email='126708766+sergimarsol@users.noreply.github.com',
    description='EKF-SLAM: joint robot pose and AprilTag landmark map estimation',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = ekf_slam.motor_control:main',
            'velocity_mapping = ekf_slam.velocity_mapping:main',
            'ekf_slam_node = ekf_slam.ekf_slam_node:main',
            'camera_tf = ekf_slam.camera_tf:main',
        ],
    },
)
