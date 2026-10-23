
from setuptools import find_packages, setup

package_name = 'node_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nayera',
    maintainer_email='nayera992004bakkar@gmail.com',
    description='ROS 2 nodes package',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'speed_control_node = node_pkg.speed_control_node:main',
            'go_to_pose_server = node_pkg.go_to_pose_server:main',
        ],
    },
)
