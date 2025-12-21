from setuptools import setup
import os
from glob import glob

setup(
    name='plan_bringup',
    version='0.0.0',
    packages=['plan_bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/plan_bringup']),
        ('share/plan_bringup', ['package.xml']),
        (os.path.join('share', 'plan_bringup', 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bzhouai',
    maintainer_email='bzhouai@todo.todo',
    description='Bringup launch files for the FUEL planners',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'exploration_supervisor = plan_bringup.exploration_supervisor:main',
        ],
    },
)
