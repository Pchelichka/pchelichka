from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
		(os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kdimitrov',
    maintainer_email='kaloyankd.06@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = controller.controller:main',
            'absolute_controller = controller.absolute_controller:main',
            'hid = controller.hid:main',
			'joystick = controller.joystick:main',
			'elrs_test = controller.elrs_test:main'
        ],
    },
)
