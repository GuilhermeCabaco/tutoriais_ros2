from setuptools import find_packages, setup

package_name = 'temperature_monitor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/temperature_monitor', ['package.xml', 'temperature_system.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,  
    maintainer='guilhermecabaco',
    maintainer_email='guilhermecabaco@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'publisher_node = temperature_monitor.publisher_node:main',
            'monitor_node = temperature_monitor.temperature_monitor_node:main',
            'enhanced_sensor_node = temperature_monitor.enhanced_sensor_node:main',
            'advanced_monitor_node = temperature_monitor.advanced_monitor_node:main',
            'logger_node = temperature_monitor.temperature_logger:main',
        ],
    },
)
