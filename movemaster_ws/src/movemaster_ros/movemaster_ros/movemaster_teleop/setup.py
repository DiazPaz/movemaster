from glob import glob

from setuptools import setup

package_name = 'movemaster_teleop'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='MoveMaster project',
    maintainer_email='maintainer@example.com',
    description='Joint jogging of MoveMaster from the ESP32 teach pendant.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'jog_node = movemaster_teleop.jog_node:main',
            'pendant_sim = movemaster_teleop.pendant_sim:main',
        ],
    },
)
