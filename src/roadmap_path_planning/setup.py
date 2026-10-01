from setuptools import find_packages, setup

package_name = 'roadmap_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
        ('share/' + package_name + '/launch', ['launch/roadmap_navigation_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergi Marsol, Yule Zhang',
    maintainer_email='126708766+sergimarsol@users.noreply.github.com',
    description='Roadmap path planning (minimum-distance and maximum-safety) with Dijkstra and AprilTag localization',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = roadmap_path_planning.motor_control:main',
            'velocity_mapping = roadmap_path_planning.velocity_mapping:main',
            'roadmap_navigation_node = roadmap_path_planning.roadmap_navigation_node:main',
            'camera_tf = roadmap_path_planning.camera_tf:main',
        ],
    },
)
