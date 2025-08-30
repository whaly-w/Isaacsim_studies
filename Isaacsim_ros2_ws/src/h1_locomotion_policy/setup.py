from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'h1_locomotion_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join('share', package_name, 'policy'), glob("policy/*.pt")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='whaly',
    maintainer_email='whalyforwork@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'h1_controller = h1_locomotion_policy.h1_fullbody_controller:main'
        ],
    },
)


