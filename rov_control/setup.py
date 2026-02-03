from setuptools import find_packages, setup

package_name = 'rov_control'

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
    maintainer='mudri',
    maintainer_email='mudri@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
        'console_scripts': [
            'trajectory_node = rov_control.trajectory_node:main',
            'controller_node = rov_control.controller_node:main',
            'dummy_rov = rov_control.dummy_rov_node:main',
        ],
    },
)
