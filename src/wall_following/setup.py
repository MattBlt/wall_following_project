from setuptools import find_packages, setup

package_name = 'wall_following'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mattblt',
    maintainer_email='baltieri.mattia04@gmail.com',
    description='Wall following for the AgileX LIMO on ROS 2: LiDAR-based wall detection, local costmap and angle/speed controller, with optional EKF localization.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
