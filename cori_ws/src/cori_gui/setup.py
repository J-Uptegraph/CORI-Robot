from setuptools import setup

package_name = 'cori_gui'

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
    description='GUI components and user interface for CORI robot',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'color_display = cori_gui.color_display:main',
        ],
    },
)