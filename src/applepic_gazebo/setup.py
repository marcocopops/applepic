import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'applepic_gazebo'


def _files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f) and not f.endswith('.gitkeep')]


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'worlds'), _files('worlds/*')),
        (os.path.join('share', package_name, 'config'), _files('config/*')),
        (os.path.join('share', package_name, 'launch'), _files('launch/*')),
        (os.path.join('share', package_name, 'models'), _files('models/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marc-Antoine Meunier',
    maintainer_email='38975865+marcocopops@users.noreply.github.com',
    description='Orchard world generation and Gazebo (gz sim) integration.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
