from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'dummy_drone_chaser'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sky',
    maintainer_email='sky@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'dummy_drone_node={package_name}.dummy_drone:main',
            f'chaser_drone_node={package_name}.chaser_drone:main'
        ],
    },
)
