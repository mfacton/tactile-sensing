from setuptools import find_packages, setup

package_name = 'soft_sense'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/launch.yaml']),
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
            'sense = soft_sense.sense:main',
            'interpret = soft_sense.interpret:main',
        ],
    },
)
