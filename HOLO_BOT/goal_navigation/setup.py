from setuptools import setup

package_name = 'goal_navigation'

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
    maintainer='sreejith',
    maintainer_email='sreejith.s@wego-robotics.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_navigation = goal_navigation.waypoint_navigation:main',
            'robot_navigation = goal_navigation.robot_navigation:main',
            'goalClient = goal_navigation.goalClient:main',
            'goalActionclient = goal_navigation.goalActionclient:main',
	    'aruco_test = goal_navigation.aruco_test:main'
        ],
    },
)
