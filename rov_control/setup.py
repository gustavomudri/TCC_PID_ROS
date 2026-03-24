from setuptools import find_packages, setup

package_name = 'tcc'

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
    maintainer_email='gustavomudri@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
entry_points={
        'console_scripts': [
            'IK = tcc.IK_node:main',
            'CAN = tcc.CAN_node:main',
            'Dummy = tcc.dummy_node:main',
        ],
    },
)
