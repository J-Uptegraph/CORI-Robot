from setuptools import setup

package_name = 'cori_simulation'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='Simulation environment and behavior modeling for CORI robot',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cori_simulator = cori_simulation.cori_simulator:main',
            'demo_display = cori_simulation.demo_display:main',
            'sensor_fusion_demo = cori_simulation.sensor_fusion_demo:main',
        ],
    },
)