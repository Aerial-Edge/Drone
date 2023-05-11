from setuptools import setup

package_name = 'drone_controller'

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
    maintainer='gruppe6',
    maintainer_email='gruppe6@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller_node = drone_controller.controller_node:main',
            'testsub = drone_controller.testsub:main',
            'tuning_node = drone_controller.tuning_node:main'
        ],
    },
)
