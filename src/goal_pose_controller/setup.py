from setuptools import find_packages, setup

package_name = 'goal_pose_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),

    data_files=[
    (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name]
    ),
    (
        'share/' + package_name,
        ['package.xml']
    ),
    (
        'share/' + package_name + '/launch',
        ['launch/goal_pose_controller.launch.py']
    ),
    (
        'share/' + package_name + '/config',
        ['config/goal_pose_controller.yaml']
    ),
    ],




    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='thusitha_madushanaka',
    maintainer_email='thusithamadushanakade@gmai.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    
    entry_points={
        'console_scripts': [
            'goal_pose_controller = goal_pose_controller.goal_pose_controller:main',
            'trajectory_reader = goal_pose_controller.trajectory_reader:main',
        ],
    },

)
