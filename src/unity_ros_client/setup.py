from setuptools import find_packages, setup

package_name = 'unity_ros_client'

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
            'cmd_publisher = unity_ros_client.cmd_publisher:main',
            'cmd_publisher_timer = unity_ros_client.cmd_publisher_timer:main',
            'pcd_service = unity_ros_client.pcd_service:main',    
        ],
    },
)
