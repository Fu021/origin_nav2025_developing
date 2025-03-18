from setuptools import find_packages, setup

package_name = 'dec_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/A.yaml','config/test.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='f021',
    maintainer_email='f021@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "root=dec_tree.root:main",
            "referee_fake=dec_tree.referee_fake:main"
        ],
    },
)
