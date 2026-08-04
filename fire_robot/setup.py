from setuptools import find_packages, setup

package_name = 'fire_robot'

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
    maintainer='sjpi',
    maintainer_email='sjpi@todo.todo',
    description='Fire fighting robot package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fire_bridge = fire_robot.fire_bridge:main'
        ],
    },
)
