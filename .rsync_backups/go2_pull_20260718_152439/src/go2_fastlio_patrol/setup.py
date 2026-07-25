from setuptools import setup

package_name = 'go2_fastlio_patrol'

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
    maintainer='unitree',
    maintainer_email='unitree@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'route_recorder = go2_fastlio_patrol.route_recorder:main',
            'waypoint_follower = go2_fastlio_patrol.waypoint_follower:main',
            'unitree_cmd_node = go2_fastlio_patrol.unitree_cmd_node:main',
            'unitree_safe_cmd_node = go2_fastlio_patrol.unitree_safe_cmd_node:main',
            'unitree_go_safe_cmd_node = go2_fastlio_patrol.unitree_go_safe_cmd_node:main',
        ],
    },
)
