from setuptools import find_packages, setup

package_name = 'coverage_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/configs', ['configs/apriltags_position.yaml']),
        ('share/' + package_name + '/launch', ['launch/coverage_navigation_node.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergi Marsol, Yule Zhang',
    maintainer_email='126708766+sergimarsol@users.noreply.github.com',
    description='Full-arena coverage with a lawnmower planner and a subsumption architecture (virtual-boundary avoidance)',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = coverage_navigation.motor_control:main',
            'velocity_mapping = coverage_navigation.velocity_mapping:main',
            'coverage_navigation_node = coverage_navigation.coverage_navigation_node:main',
            'camera_tf = coverage_navigation.camera_tf:main',
        ],
    },
)
