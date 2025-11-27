from setuptools import find_packages, setup

package_name = 'kinova_control'

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
    maintainer='pc',
    maintainer_email='jyw010704@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'kinova_driver_node = kinova_control.kinova_driver_node:main',
            'vision_node = kinova_control.vision_node:main',
            'pose_calculator_node = kinova_control.pose_calculator_node:main',
            'kinova_client_node = kinova_control.kinova_client_node:main',
            'kinova_main_node = kinova_control.kinova_main_node:main',
        ],
    },
)
