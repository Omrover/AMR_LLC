from setuptools import find_packages, setup

setup(
    name='nxp_mqtt_lab',
    version='2.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/nxp_mqtt_lab']),
        ('share/nxp_mqtt_lab', ['package.xml']),
    ],
    install_requires=['setuptools', 'paho-mqtt>=1.6,<3'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Course instructor',
    maintainer_email='instructor@example.org',
    description='MQTT encoder bridge, simulator and student monitor.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'bridge = nxp_mqtt_lab.bridge:main',
        'simulator = nxp_mqtt_lab.simulator:main',
        'monitor = nxp_mqtt_lab.monitor:main',
    ]},
)
