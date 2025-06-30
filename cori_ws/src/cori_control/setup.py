from setuptools import setup

setup(
    name='cori_control',
    version='0.0.0',
    packages=['cori_control'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cori_control']),
        ('share/cori_control', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI control Package',
    license='Apache License 2.0',
    entry_points={'console_scripts': ['joint_controller = cori_control.joint_controller:main']},
)

# Add joint controller executable
setup.entry_points['console_scripts'].append('joint_controller = cori_control.joint_controller:main')
