from setuptools import find_packages, setup
import os
import glob
package_name = 'storagy_exhibition_ui'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch','*.launch.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sineunji',
    maintainer_email='sineunji@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'main_ui = storagy_exhibition_ui.RobotUi:main',
            'scanner = storagy_exhibition_ui.scanner:main',
            'database = storagy_exhibition_ui.database:main',
            'siren = storagy_exhibition_ui.robot_voice:main',  ,
            'graph = storagy_exhibition_ui.grape_data:main'
        
        ],
    },
)
