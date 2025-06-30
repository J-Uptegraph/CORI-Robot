from setuptools import setup

setup(
    name='cori_core',
    version='0.0.0',
    packages=['cori_core'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cori_core']),
        ('share/cori_core', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI core Package',
    license='Apache License 2.0',
    entry_points={'console_scripts': []},
)
