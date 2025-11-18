import os
from glob import glob
from setuptools import setup

package_name = 'control_rbkairos'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
           ['resource/control_rbkairos']),
        ('share/control_rbkairos', ['package.xml']),
        (os.path.join('share', 'control_rbkairos', 'launch'), glob('launch/*')),
        (os.path.join('share', 'control_rbkairos', 'worlds'), glob('worlds/*')),
        (os.path.join('share', 'control_rbkairos', 'urdf'), glob('urdf/*')),
        (os.path.join('share', 'control_rbkairos', 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thuwanon',
    maintainer_email='thuwanon@todo.todo',
    description='RBKairos robot control package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = control_rbkairos.move_robot:main',
            'mecanum_move = control_rbkairos.mecanum_move:main',
        ],
    },
)
