from setuptools import setup

package_name = 'robot'
data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name + '/launch', ['launch/sim.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/sim_teleop.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/vlm_control_sim.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/vlm_control_ugv.py']))
data_files.append(('share/' + package_name + '/launch', ['launch/webots_world.py']))
data_files.append(('share/' + package_name + '/worlds', ['worlds/my_world.wbt']))
data_files.append(('share/' + package_name + '/resource', ['resource/my_robot.urdf']))
data_files.append(('share/' + package_name + '/resource', ['resource/config.yaml']))
data_files.append(('share/' + package_name, ['package.xml']))

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools', 'opencv-python', 'aiohttp', 'Pillow', 'pyserial', 'numpy', 'requests'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user.name@mail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    scripts=[
        'scripts/webots_driver',
        'scripts/vlm_control',
        'scripts/ugv_driver',
        'scripts/talker',
        'scripts/listener',
        'scripts/camera',
        'scripts/keyboard_teleop',
    ],
)