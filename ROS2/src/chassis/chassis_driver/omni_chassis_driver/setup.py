from setuptools import setup, find_packages
import os
import glob

package_name = 'omni_chassis_driver'

setup(
    name        = package_name,
    version     = '0.0.0',
    packages    = find_packages(exclude=['test', 'launch', 'resource']),
    data_files  = [
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
    ],
    install_requires    = ['setuptools'],
    zip_safe            = True,
    maintainer          = 'root',
    maintainer_email    = 'root@todo.todo',
    description         = 'TODO: Package description',
    license             = 'TODO: License declaration',
    tests_require       = ['pytest'],
    entry_points        = {
        'console_scripts': [
             "omni_base_controller_global_actual_node      = omni_base_controller.base_controller_global_actual:main",
             "omni_base_controller_global_simulation_node  = omni_base_controller.base_controller_global_simulation:main",
             "omni_base_controller_local_actual_node       = omni_base_controller.base_controller_local_actual:main",
             "omni_base_controller_local_simulation_node   = omni_base_controller.base_controller_local_simulation:main",
             "omni_base_driver_node            = omni_base_driver.base_driver:main",
             "omni_simulation_base_driver_node = omni_simulation_base_driver.simulation_base_driver_zmq:main",
        ],
    },
)
