import os
from glob import glob
from setuptools import setup

package_name = 'img_processing'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*.tflite')),
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
            'videostream_node = img_processing.videostream_node:main',
            'detect_node = img_processing.detect_node:main',
            'testsub = img_processing.testsub:main',
        ],
    },
)
