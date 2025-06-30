from setuptools import setup

setup(
    name='cori_tools',
    version='0.0.0',
    packages=['cori_tools'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/cori_tools']),
        ('share/cori_tools', ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johnathan Uptegraph',
    maintainer_email='jwuptegraph@gmail.com',
    description='CORI tools Package',
    license='Apache License 2.0',
    entry_points={'console_scripts': []},
)
