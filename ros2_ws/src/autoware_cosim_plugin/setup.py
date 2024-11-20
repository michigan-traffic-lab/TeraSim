from setuptools import find_packages, setup

package_name = 'autoware_cosim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Zhijie Qiao',
    maintainer_email='zhijieq@umich.edu',
    description='\
        Receive trajectory from redis server and convert to ros2 message\
        Convert vehicle state information from ros2 message and send to redis server\
    ',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autoware_cosim_plugin = autoware_cosim_plugin.autoware_cosim_plugin:main',
            'autoware_tls_plugin = autoware_cosim_plugin.autoware_tls_plugin:main',
            'autoware_dummy_grid = autoware_cosim_plugin.autoware_dummy_grid:main',
            'autoware_cav_cmd = autoware_cosim_plugin.autoware_cav_cmd:main',
        ],
    },
)
