from setuptools import setup

package_name = 'storagy_human_tracking'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your package description',
    license='Your license',
    entry_points={
        'console_scripts': [
            'BlueToothRobot = robot_node_pkg.BlueToothRobot:main',
            'final_human_tracking = robot_node_pkg.final_human_tracking:main',
            'human_detection_yolov8n = robot_node_pkg.human_detection_yolov8n:main',
            'BlueTooth = robot_node_pkg.BlueTooth:main',
        ],
    },
)





