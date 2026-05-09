import os
from glob import glob
from setuptools import setup

package_name = 'inspection_robot'

# Recursively pick up every FILE (not dir) under models/ so the Gazebo asset
# tree (model.config, model.sdf, materials/scripts, materials/textures) is
# preserved in the install share dir grouped by parent directory.
def tree(root):
    by_dir = {}
    for path in glob(root + '/**/*', recursive=True):
        if not os.path.isfile(path):
            continue
        parent = os.path.dirname(path)
        by_dir.setdefault(parent, []).append(path)
    return [(f'share/{package_name}/{d}', files) for d, files in by_dir.items()]

data_files = [
    ('share/ament_index/resource_index/packages',
     ['resource/' + package_name]),
    (f'share/{package_name}', ['package.xml']),
    (f'share/{package_name}/launch', glob('launch/*.launch.py')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
    (f'share/{package_name}/worlds', glob('worlds/*.world')),
]
data_files += tree('models')

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='izzatbek',
    maintainer_email='murodjonovizzatbek@gmail.com',
    description='Autonomous inspection robot in Gazebo: waypoint follower + ArUco detection + report logger.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detector = inspection_robot.aruco_detector:main',
            'waypoint_follower = inspection_robot.waypoint_follower:main',
            'report_logger = inspection_robot.report_logger:main',
        ],
    },
)
