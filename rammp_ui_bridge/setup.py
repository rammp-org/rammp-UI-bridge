from setuptools import find_packages, setup

package_name = 'rammp_ui_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ui_bridge.launch.py']),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='Liyun Guo',
    maintainer_email='guo@assistivetech.dev',
    description=(
        'Bridges an Unreal Engine touchscreen UI to the RAMMP Orchestrator '
        "over the Orchestrator's public ROS2 interface."
    ),
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ui_bridge_node = rammp_ui_bridge.ros.ui_bridge_node:main',
        ],
    },
)
