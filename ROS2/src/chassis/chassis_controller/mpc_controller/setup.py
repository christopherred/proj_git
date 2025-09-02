# from setuptools import find_packages, setup
# import os
# import glob

# package_name = 'diff_chassis_driver'

# setup(
#     name        = package_name,
#     version     = '0.0.0',
#     packages    = find_packages(exclude=['test']),
#     data_files  = [
#         ('share/ament_index/resource_index/packages',   ['resource/' + package_name]),
#         ('share/' + package_name,                       ['package.xml']),
#         (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
#         (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml')),
#     ],
#     install_requires = ['setuptools'],
#     zip_safe         = True,
#     maintainer       = 'root',
#     maintainer_email = 'hankunjiang@outlook.com',
#     description      = 'TODO: Package description',
#     license          = 'TODO: License declaration',
#     tests_require    = ['pytest'],
#     entry_points     = {
#         'console_scripts': [
#             "diff_base_driver_node      = base_driver.base_driver_direct:main",
#             "diff_base_controller_node  = base_controller.base_controller:main",
#         ],
#     },
# )


from setuptools import setup, find_packages
import os
import glob

package_name = 'mpc_controller'

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
             "mpc_controller_node         = mpc_controller.mpc_controller:main",
             "traj_publisher_node         = mpc_controller.traj_publisher:main",
        ],
    },
)
