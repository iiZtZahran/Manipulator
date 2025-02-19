from setuptools import find_packages, setup

package_name = 'manipulator_py'

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
    maintainer='zahran',
    maintainer_email='typhon152@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 
            'simple_publisher = arduinobot_py.simple_publisher:main',
             'simple_subscriber = arduinobot_py.simple_subscriber:main',
             'simple_parameter = arduinobot_py.simple_parameter:main',
        ],
    },
)
