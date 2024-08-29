from setuptools import find_packages, setup

package_name = 'storagy_apriltag_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/aruco_tracking_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='storagy',
    maintainer_email='storagy@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = storagy_apriltag_tracking.aruco_node:main',
            'cmd_node = storagy_apriltag_tracking.aruco_yolo_cmd_node:main',
        ],
    },
)
