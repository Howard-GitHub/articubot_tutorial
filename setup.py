from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'articubot_one'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'description'), glob('description/*')), 
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')), 
        (os.path.join('share', package_name, 'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='howard.l.general@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_twist_node = articubot_one.test_twist:main",
            "odom_subscribe_node = articubot_one.odom_subscriber:main",
            "pose_subscriber_node = articubot_one.pose_subscriber:main",
            "pure_pursuit_dubins_node = articubot_one.pure_pursuit_dubins:main",
            "pure_pursuit_proto_node = articubot_one.pure_pursuit_proto:main",
            "set_orientation_node = articubot_one.set_vehicle_heading:main"
        ],
    },
)