from setuptools import find_packages, setup

package_name = 'wear_advisor'

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
    maintainer='Yuken Ro',
    maintainer_email='yuken.lu3@gmail.com',
    description='ROS2 package that suggests clothing based on weather and temperature input.',
    license='BSD-3-Clause',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
             'wear_node = wear_advisor.wear_node:main',
             'wear_pub = wear_advisor.wear_pub:main',
             'wear_print = wear_advisor.wear_print:main',
        ],
    },
)
