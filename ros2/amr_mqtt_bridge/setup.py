from glob import glob

from setuptools import find_packages, setup

package_name = 'amr_mqtt_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'paho-mqtt>=1.6,<3'],
    zip_safe=True,
    maintainer='Omrover',
    maintainer_email='sanduich13@gmail.com',
    description='Bridge ROS 2 <-> MQTT con el controlador Tlaloc S32K344.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'bridge = amr_mqtt_bridge.bridge_node:main',
        'mcu_emulator = amr_mqtt_bridge.mcu_emulator:main',
    ]},
)
