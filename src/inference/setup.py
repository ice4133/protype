from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'inference'

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
    maintainer='eai',
    maintainer_email='wangyong233@midea.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'joint_base_node = inference.joint_base_node:main',
            'camera_node = inference.camera_node:main',
            'intervention_relay_node = inference.intervention_relay_node:main',
            'mock_inference_pub = inference.mock_inference_pub:main',
            'mock_master_pub = inference.mock_master_pub:main',
            'hdf5_playback_pub = inference.hdf5_playback_pub:main',
        ],
    },
)
