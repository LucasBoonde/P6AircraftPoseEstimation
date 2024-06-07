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
    maintainer='anders',
    maintainer_email='anders@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'lidar_reg = lidar_reg.lidar_reg_for_linear_test:main',
            'test = lidar_reg.mikkelcookershit:main', #DEN VIRKER!!
            #'bug2 = lidar_reg.ODOM:main',
            #'lucasbewcas = lidar_reg.Bug2Working:main',
            #'sub = lidar_reg.Subscriber:main',
            'pub = lidar_reg.Publisher', #DEN VIRKER!!!
            #'subscriber = lidar_reg.Wsubscriber:main',
            #'backup = lidar_reg.Backup:main',
            #'Backup2 = lidar_reg.Backup2:main',
            
            

        ],
    },
)
