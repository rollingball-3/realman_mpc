from setuptools import setup

package_name = 'gazebo_mpc_interface'

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
    maintainer='qiufangzhou',
    maintainer_email='fqiu@a4x.io',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_mpc_interface = gazebo_mpc_interface.gazebo_mpc_interface:main',
        ],
    },
)
