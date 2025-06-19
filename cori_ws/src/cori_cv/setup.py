from setuptools import setup
import os
from glob import glob

package_name = 'cori_cv'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add this line to include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI Computer Vision Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'laundry_color_detector = cori_cv.laundry_color_detector:main',
            'simple_color_detector = cori_cv.simple_color_detector:main',
	        'color_display = cori_cv.color_display:main', 
        ],
    },
)
