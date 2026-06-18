from setuptools import setup

package_name = 'robot'
_share = f'share/{package_name}'

_LAUNCH = [
    'launch/behaviour_recorder.py',
    'launch/robot_behavmodel.py',
    'launch/robot_teleop.py',
    'launch/robot_teleop_aug.py',
    'launch/sensing_control.py',
    'launch/sim_behavmodel.py',
    'launch/sim_teleop.py',
    'launch/sim_teleop_augmented.py',
    'launch/vlm_control_sim.py',
    'launch/vlm_control_ugv.py',
    'launch/vlm_monitor.py',
    'launch/webots_world.py',
]
_WORLDS = ['worlds/my_world.wbt', 'worlds/track.wbt']
_RESOURCE = [
    'resource/my_robot.urdf',
    'resource/config.yaml',
    'resource/vlm_monitor.perspective',
]

data_files = [
    ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
    (f'{_share}/launch', _LAUNCH),
    (f'{_share}/worlds', _WORLDS),
    (f'{_share}/resource', _RESOURCE),
    (_share, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=[
        'setuptools',
        'opencv-python',
        'aiohttp',
        'Pillow',
        'pyserial',
        'numpy',
        'requests',
        'onnx',
        'onnxruntime',
    ],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlm_control = robot.vlm_control:main',
            'ugv_driver = robot.ugv_driver:main',
            'talker = robot.talker:main',
            'listener = robot.listener:main',
            'camera = robot.camera:main',
            'keyboard_teleop = robot.keyboard_teleop:main',
            'behaviour_recorder = robot.behaviour_recorder:main',
            'behaviour_control = robot.behaviour_control:main',
            'vel_augmenter = robot.vel_augmenter:main',
        ],
    },
)
