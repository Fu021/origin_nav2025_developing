from setuptools import find_packages, setup
from glob import glob
import os
package_name = 'dec_tree'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'PyYAML',
    ],
    zip_safe=True,
    maintainer='f021',
    maintainer_email='f021@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "root=dec_tree.root:main",
            "root_simple=dec_tree.rootsimple:main",
            "root_radical=dec_tree.rootradical:main",
            "referee_fake=dec_tree.referee_fake:main",
        ],
    },
)
