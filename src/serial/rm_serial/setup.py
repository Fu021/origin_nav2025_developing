from setuptools import find_packages, setup

package_name = 'rm_serial'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/' + 'rm_serial.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nuc3',
    maintainer_email='nuc3@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "rm_serial_subpub=rm_serial.rm_serial_subpub_old:main",
            "send_cmd_vel=rm_serial.send_cmd_vel:main"
        ],
    },
)
