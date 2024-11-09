from setuptools import find_packages, setup

package_name = 'create3_navigation_task'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/target_pose.yaml', 'config/nav2_params.yaml', 'config/target_pose.yaml']),
          # Add this line
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='romain',
    maintainer_email='romain-robe@hotmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'navigate_create3 = create3_navigation_task.navigate_create3:main',
            'goal_handling = create3_navigation_task.goal_handling:main'
        ],
    },
)
