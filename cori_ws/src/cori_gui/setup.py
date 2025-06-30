from setuptools import setup

setup(
    name='cori_gui',
    version='0.0.0',
    packages=['cori_gui'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cori_gui']),
        ('share/cori_gui', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI gui Package',
    license='Apache License 2.0',
    entry_points={'console_scripts': []},
)
