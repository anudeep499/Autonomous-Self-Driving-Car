from setuptools import find_packages, setup

package_name = 'goal_follower'

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
    maintainer='tne',
    maintainer_email='joebneal1@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_follower_v1 = goal_follower.goal_follower_v1:main',
            'goal_follower_v2 = goal_follower.goal_follower_v2:main',
            'path_follower_v1 = goal_follower.path_follower_v1:main',
            'path_follower_v2 = goal_follower.path_follower_v2:main'
        ],
    },
)
