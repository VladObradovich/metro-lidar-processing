from glob import glob

from setuptools import find_packages, setup


package_name = 'metro_lidar_processing'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Hackathon Team',
    maintainer_email='team@example.com',
    description='ROS 2 processing nodes for metro tunnel lidar data.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'depth_image = metro_lidar_processing.depth_image_node:main',
        ],
    },
)
