from setuptools import find_packages, setup

package_name = 'fpv_lab2'

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
    maintainer='tetianadanylo',
    maintainer_email='tetianadanylo@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'flight_test = fpv_lab2.main:main',
        'flight_test_maze = fpv_lab2.main_maze:main',
        'flight_variant6 = fpv_lab2.variant6_node:main',
        'flight_variant6_high = fpv_lab2.variant6_high_node:main',
        'flight_variant6_maze = fpv_lab2.variant6_maze_node:main',
        ],
    },
)
