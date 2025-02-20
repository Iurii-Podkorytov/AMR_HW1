from setuptools import find_packages, setup

package_name = 'my_robot_controllers'

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
    maintainer='yura',
    maintainer_email='i.podkorytov@innopolis.university',
    description='Simple controllers for differential drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = my_robot_controllers.keyboard_controller:main',
            'pure_pursuit_control = my_robot_controllers.pure_pursuit_controller:main',
            'proportional_control = my_robot_controllers.proportional_controller:main',
            'stanley_control = my_robot_controllers.stanley_controller:main',
            'mpc = my_robot_controllers.model_predictive_controller:main',
            'plotter = my_robot_controllers.plotter:main',
        ],
    },
)
