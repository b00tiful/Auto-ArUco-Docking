from setuptools import find_packages, setup

package_name = 'aruco_docking'

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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "aruco_pose_estimation = aruco_docking.aruco_pose_estimation:main",
            "docking_controller = aruco_docking.docking_controller:main",
            "video_capture = aruco_docking.video_capture:main"
        ],
    },
)
