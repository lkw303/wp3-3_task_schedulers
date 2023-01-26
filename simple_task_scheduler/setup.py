import os
from glob import glob
from setuptools import setup

package_name = 'simple_task_scheduler'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.*')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kai',
    maintainer_email='lum_kai_wen@artc.a-star.edu.sg',
    description='A simple task scheduler',
    license='Apache License Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_task_scheduler=simple_task_scheduler.main:main',
            'test_command=simple_task_scheduler.test_command:main'
        ],
    },
)
