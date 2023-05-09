from setuptools import setup

package_name = 'config4'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'sensor_msgs', 'std_msgs', 'cv_bridge', 'opencv4'],
    zip_safe=True,
    maintainer='vaffe',
    maintainer_email='vaffe@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detection = config4.object_detection:main',
            'camera_capture = config4.camera_capture:main',
        ],
    },
)
