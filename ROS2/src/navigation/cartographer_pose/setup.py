from setuptools import find_packages, setup
import os, glob

package_name = 'cartographer_pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.lua')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.lua')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='hankunjiang@outlook.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_to_pose_node = cartographer_pose.tf_to_pose:main',
        ],
    },
)
