from glob import glob
from setuptools import find_packages, setup

setup(
    name='wheelz_ros', version='0.1.0', packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/wheelz_ros']),
        ('share/wheelz_ros', ['package.xml']),
        ('share/wheelz_ros/launch', glob('launch/*.launch.py')),
        ('share/wheelz_ros/config', glob('config/*.yaml')),
        ('share/wheelz_ros/models', glob('models/*.npz') + glob('models/*.json') + glob('models/*.md')),
    ],
    install_requires=['setuptools'], extras_require={'test': ['pytest']}, zip_safe=True,
    maintainer='Donovan', maintainer_email='wheelz@example.com',
    description="ROS 2 host bridge for Wheelz's unchanged ESP HTTP firmware",
    license='Proprietary',
    entry_points={'console_scripts': [
        'driver = wheelz_ros.driver:main',
        'camera = wheelz_ros.camera:main',
        'mock_esp = wheelz_ros.mock_esp:main',
        'policy = wheelz_ros.policy_node:main',
        'app_gateway = wheelz_ros.app_gateway:main',
    ]},
)
