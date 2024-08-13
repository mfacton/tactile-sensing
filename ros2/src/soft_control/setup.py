from setuptools import find_packages, setup

package_name = 'soft_control'

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
    maintainer='mfact',
    maintainer_email='mfacton1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'center_impedence = soft_control.center_impedence:main'
            'ring_impedence = soft_control.ring_impedence:main'
            'disk_impedence = soft_control.disk_impedence:main'
        ],
    },
)