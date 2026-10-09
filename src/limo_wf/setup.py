from setuptools import find_packages, setup

package_name = 'limo_wf'

setup(
    name=package_name,
    version='0.0.0',
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
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'wall_detection = limo_wf.wall_detection_node:main',
            'local_grid = limo_wf.local_grid_node:main',
            'wall_follow_controller = limo_wf.wall_follow_controller_node:main',
            'return_to_start = limo_wf.return_to_start_node:main',
        ],
    },
)
