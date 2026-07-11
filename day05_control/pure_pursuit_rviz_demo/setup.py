from glob import glob
from setuptools import setup

package_name = 'pure_pursuit_rviz_demo'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='navigation_teach',
    maintainer_email='demo@example.com',
    description='Lightweight Pure Pursuit RViz demo with a virtual URDF car.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'path_publisher = pure_pursuit_rviz_demo.path_publisher:main',
            'pure_pursuit_controller = pure_pursuit_rviz_demo.pure_pursuit_controller:main',
            'virtual_robot = pure_pursuit_rviz_demo.virtual_robot:main',
        ],
    },
)
