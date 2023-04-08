from setuptools import setup

package_name = 'py_hubbot_main_controller_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jetson',
    maintainer_email='antakidavid@gmail.com',
    description='For communicating with minibots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'hubbot_stat_pub = py_hubbot_main_controller_node.hubbot_status_publisher:main',
                'minibot_stat_sub = py_hubbot_main_controller_node.minibot_status_subscriber:main',
                'hubbot_main_controller_node = py_hubbot_main_controller_node.hubbot_main_controller_node:main',
        ],
    },
)
