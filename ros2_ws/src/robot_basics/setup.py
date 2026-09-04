from setuptools import find_packages, setup

package_name = 'robot_basics'

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
    maintainer='wolker',
    maintainer_email='wolker@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'publisher = robot_basics.publisher:main',
		'subscriber = robot_basics.subscriber:main',
		'publisher_best_effort = robot_basics.publisher_best_effort:main',
		'subscriber_reliable = robot_basics.subscriber_reliable:main',
		'publisher_reliable = robot_basics.publisher_reliable:main',
		'subscriber_best_effort = robot_basics.subscriber_best_effort:main'
        ],
    },
)
