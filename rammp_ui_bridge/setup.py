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
    install_requires=[
        'setuptools',
        'websockets',
        # Pinned to a release tag (not the floating default branch) -- v0.2.1
        # is the first release where ramms-tools/issues/13 is fixed: the
        # NumPy floor is numpy>=1.21 (matches Ubuntu 22.04/ROS Humble's
        # system python3-numpy 1.21.5) and setuptools is capped <80, so
        # installing this no longer upgrades NumPy and breaks the system
        # python3-scipy ROS code elsewhere depends on.
        'ramms-tools @ git+https://github.com/rammp-org/ramms-tools.git@v0.2.1',
    ],
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
            'mock_ue = rammp_ui_bridge.mocks.mock_ue:main',
            'test_request_user_input_node = rammp_ui_bridge.mocks.test_request_user_input_node:main',
        ],
    },
)
