import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gece_es'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ambatron',
    maintainer_email='ambatron@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'process_telemetry = gece_es.processTelemetry:main'
        ],
    },
)
