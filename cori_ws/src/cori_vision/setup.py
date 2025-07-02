from setuptools import setup
import os
from glob import glob

package_name = 'cori_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', ['config/color_piles.yaml']),
        ('lib/' + package_name, []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='Computer vision and object detection for CORI robot',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = cori_vision.object_detection:main',
            'simple_color_detector = cori_vision.simple_color_detector:main',
            'laundry_color_detector = cori_vision.laundry_color_detector:main',
        ],
    },
)