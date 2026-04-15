from setuptools import find_packages, setup

package_name = 'robot_control'

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
    maintainer='jinju',
    maintainer_email='wnwlswn23@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'test_node = robot_control.test:main',
            'joint_state_publisher = robot_control.joint_state_publisher:main',
            'quest_teleop = robot_control.quest_teleop:main',
            'test_servo = robot_control.test_servo:main',
        ],
    },
)
