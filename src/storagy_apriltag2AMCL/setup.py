from setuptools import setup

package_name = 'storagy_apriltag2AMCL'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools', 'opencv-python', 'apriltag', 'cv_bridge'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='AprilTag ROS 2 package',
    license='MIT',
    entry_points={
        'console_scripts': [
            'apriltag_node = storagy_apriltag2AMCL.ApriltagDetectionNode:main',
            'CameraCalibrationNode = storagy_apriltag2AMCL.DepthCameraCalibrationNode:main',
            'carrier_node = storagy_apriltag2AMCL.Carrier:main',
            'take_parking_node= storagy_apriltag2AMCL.TagParkingController:main',

         ],
     },
 )
