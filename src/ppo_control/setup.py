from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ppo_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.onnx')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pavel',
    maintainer_email='pavel@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ppo_node = ppo_control.ppo_node:main',
        ],
    },
)
