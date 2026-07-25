from setuptools import setup

package_name = 'go2_loop_backend'

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
    maintainer_email='unitree@example.com',
    description='GO2 loop closure backend tools',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyframe_saver = go2_loop_backend.keyframe_saver:main',
            'offline_keyframe_extractor = go2_loop_backend.offline_keyframe_extractor:main',
            'build_raw_map = go2_loop_backend.build_raw_map:main',
            'scan_context_detector = go2_loop_backend.scan_context_detector:main',
            'pose_graph_optimizer = go2_loop_backend.pose_graph_optimizer:main',
            'dynamic_map_filter = go2_loop_backend.dynamic_map_filter:main',
            'sliding_window_static_filter = go2_loop_backend.sliding_window_static_filter:main',
            'export_registered_cloud_map = go2_loop_backend.export_registered_cloud_map:main',
            'level_pcd = go2_loop_backend.level_pcd:main',
            'pcd_to_nav2_map = go2_loop_backend.pcd_to_nav2_map:main',
            'pcd_to_nav2_map_fast = go2_loop_backend.pcd_to_nav2_map_fast:main',
        ],
    },
)
