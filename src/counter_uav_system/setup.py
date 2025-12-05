from setuptools import setup
import os
from glob import glob

package_name = 'counter_uav_system'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    # --- CHANGE HERE: Add matplotlib>=3.9.0 ---
    install_requires=[
        'setuptools',
        'matplotlib>=3.9.0',  # Fixes the NumPy 2.x incompatibility
        'numpy>=2.0.0'        # Explicitly allows the new NumPy
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='UAV Counter System Simulation',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'drone_sim_node = counter_uav_system.drone_sim_node:main',
            'sensor_sim_node = counter_uav_system.sensor_sim_node:main',
            'particle_filter_node = counter_uav_system.particle_filter_node:main',
            'data_plotter = counter_uav_system.Data_plotter:main',
            'bbox_adapter_node = counter_uav_system.bbox_adapter_node:main',
        ],
    },
)