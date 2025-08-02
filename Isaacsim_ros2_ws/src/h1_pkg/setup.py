from setuptools import find_packages, setup

package_name = 'h1_pkg'

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
    maintainer='whaly',
    maintainer_email='whalyforwork@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'proto_odom_cal = h1_pkg.proto_odom_cal:main',
            'proto_teleop = h1_pkg.proto_teleop:main',
            'robot_feedback_control = h1_pkg.robot_feedback_control:main',
        ],
    },
)
