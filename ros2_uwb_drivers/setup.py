from setuptools import find_packages, setup

package_name = 'ros2_uwb_drivers'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/uwb_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/uwb_driver.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='anand',
    maintainer_email='anandchowdary2005@gmail.com',
    description='Generic UWB hardware driver for serial-based devices.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'uwb_serial_driver = ros2_uwb_drivers.uwb_serial_driver_node:main',
        ],
    },
)
