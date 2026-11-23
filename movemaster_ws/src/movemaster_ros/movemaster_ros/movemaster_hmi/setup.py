from pathlib import Path

from setuptools import setup

package_name = 'movemaster_hmi'
# Images of the intro and of the hidden scene, loaded next to hmi_movemaster.py.
assets = [str(path.relative_to(package_name))
          for path in sorted(Path(package_name, 'assets').rglob('*.png'))]

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_data={package_name: assets},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements.txt']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='MoveMaster project',
    maintainer_email='maintainer@example.com',
    description='Touch screen HMI of MoveMaster and its API to the robot over ROS 2.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hmi = movemaster_hmi.hmi_movemaster:main',
        ],
    },
)
