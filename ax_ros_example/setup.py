from setuptools import setup

package_name = 'ax_ros_example'

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
    maintainer='brian',
    maintainer_email='bpwilcox@eng.ucsd.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_ax_server = ax_ros_example.ax_trial_server:main',
            'simple_ax_client= ax_ros_example.ax_trial_client:main',
        ],
    },
)
