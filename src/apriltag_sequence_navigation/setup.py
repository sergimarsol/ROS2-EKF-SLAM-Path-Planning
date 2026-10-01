from setuptools import find_packages, setup

package_name = 'apriltag_sequence_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/sequence_navigation.launch.py'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sergi Marsol, Yule Zhang',
    maintainer_email='126708766+sergimarsol@users.noreply.github.com',
    description='Calibrated closed-loop waypoint navigation with AprilTag pose corrections',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'motor_control = apriltag_sequence_navigation.motor_control:main',
            'sequence_navigation = apriltag_sequence_navigation.sequence_navigation:main',
        ],
    },
)
