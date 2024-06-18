from setuptools import find_packages, setup

package_name = 'vision_model_package'

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
    maintainer='kodenfly123',
    maintainer_email='kodenfly123@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'project_main_script = vision_model_package.ProjectMainScript:main',
            'xyz_subscriber = vision_model_package.XYZSubscriber:main'
        ],
    },
)
