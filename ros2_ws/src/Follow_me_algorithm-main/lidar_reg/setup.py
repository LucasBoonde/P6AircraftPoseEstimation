from setuptools import setup

package_name = 'lidar_reg'

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
    maintainer='Kian',
    maintainer_email='kiannayebi@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sub = lidar_reg.subscriber:main', 
            'pub = lidar_reg.Publisher', 
            'waypoint = lidar_reg.waypoint:main' 
        ],
    },
)
