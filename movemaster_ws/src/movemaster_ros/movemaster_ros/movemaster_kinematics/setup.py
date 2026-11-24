from setuptools import setup

package_name = 'movemaster_kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='MoveMaster project',
    maintainer_email='maintainer@example.com',
    description='Forward and closed-form inverse kinematics of MoveMaster.',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinematics = movemaster_kinematics.cli:main',
        ],
    },
)
