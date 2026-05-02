from setuptools import setup
from glob import glob
import os

package_name = 'amr_mission_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('lib/python3.10/site-packages/' + package_name + '/templates', glob(package_name + '/templates/*')),
        ('lib/python3.10/site-packages/' + package_name + '/static', glob(package_name + '/static/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hungubuntu',
    maintainer_email='hungubuntu@todo.todo',
    description='AMR mission manager web UI',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_mission_server = amr_mission_manager.web_mission_server:main',
        ],
    },
)