from setuptools import find_packages, setup

## Added Libraies
import os
from glob import glob

package_name = 'test_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.py")),
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
            # Pub & Sub nodes
            'talker = test_pkg.test_publisher:main',
            'listener = test_pkg.test_subscriber:main',
            'subPub = test_pkg.test_subPub:main',
            'generic_listener = test_pkg.test_generic_subscriber:main',
            'msg_talker = test_pkg.test_msg:main',
            
            # Service nodes
            'service_server = test_pkg.test_service_server:main',
            'service_client = test_pkg.test_service_client:main',
            
            # Action nodes
            'action_server = test_pkg.test_action_server:main',
            'action_client = test_pkg.test_action_client:main',
            'turtlesim_draw_server = test_pkg.turtlesim_draw_action_server:main',
        ],
    },
)
