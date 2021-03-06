from setuptools import setup

package_name = 'youbot_fake_moveit'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ctruillet',
    maintainer_email='clement@ctruillet.eu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controlArm = youbot_fake_moveit.controlArm:main',
            'controlBase = youbot_fake_moveit.controlBase:main',
        ],
    },
)
