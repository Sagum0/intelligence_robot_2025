from setuptools import find_packages, setup

package_name = 'final_kinova'

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
            'vision = final_kinova.vision_node:main',
            'driver = final_kinova.kinova_driver_node:main',
            'client = final_kinova.kinova_client_node:main',
            'main = final_kinova.kinova_main_node:main',
        ],
    },
)
