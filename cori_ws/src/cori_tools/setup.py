from setuptools import setup

package_name = 'cori_tools'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', ['scripts/cori_info_collector.sh']),
        ('share/' + package_name + '/config', ['config/cori_ignition_database.json']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='Development tools and utilities for CORI robot',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ignition_integration = cori_tools.cori_ignition_integration:main',
        ],
    },
)