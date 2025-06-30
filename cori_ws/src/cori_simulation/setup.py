from setuptools import setup

setup(
    name='cori_simulation',
    version='0.0.0',
    packages=['cori_simulation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cori_simulation']),
        ('share/cori_simulation', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI simulation Package',
    license='Apache License 2.0',
    entry_points={'console_scripts': []},
)
