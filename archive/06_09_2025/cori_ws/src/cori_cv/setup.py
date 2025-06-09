from setuptools import setup
from glob import glob
import os

package_name = 'cori_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='johnnyup2@gmail.com',
    description='Laundry color and clothing type detection for CORI using OpenCV and ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laundry_color_detector = cori_cv.nodes.laundry_color_detector:main',
        ],
    },
)
