from setuptools import setup
import os
from glob import glob

package_name = 'arx_cmd_pub'

setup(
    name=package_name,
    version='0.15.3',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='',
    maintainer_email='',
    description='',
    license='Apache 2.0',
    entry_points={
        'console_scripts': [
            'arx_cmd_pub = arx_cmd_pub.cmd_processor:main',
            'lift_node = arx_cmd_pub.lift_node:main',
        ],
    },
)
