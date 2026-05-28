from setuptools import find_packages, setup

package_name = 'ltm2985_uart'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ltm2985_uart.launch.py']),
        ('share/' + package_name + '/config', ['config/ltm2985_uart.params.yaml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='ROS 2 UART reader for Arduino LTM2985 JSON frames',
    license='MIT',
    entry_points={
        'console_scripts': [
            'ltm2985_uart_node = ltm2985_uart.node:main',
        ],
    },
)
