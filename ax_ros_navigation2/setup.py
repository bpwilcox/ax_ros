from setuptools import setup

package_name = 'ax_ros_navigation2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    # py_modules=[
    #     'gazebo_interface',
    #     'nav_ax_trial_runner',
    # ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian',
    maintainer_email='bpwilcox@eng.ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_ax_trial_runner = ax_ros_navigation2.nav_ax_trial_runner:main',
        ],
    },
)
