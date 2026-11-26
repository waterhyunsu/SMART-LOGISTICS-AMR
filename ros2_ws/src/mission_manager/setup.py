from setuptools import setup

package_name = 'mission_manager'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='SMART-LOGISTICS AMR',
    maintainer_email='waterhyunsu@gmail.com',
    description='Mission state and plan acceptance.',
    license='TODO',
    entry_points={'console_scripts': [
        'mission_manager = mission_manager.node:main',
        'mission_web_gateway = mission_manager.web_gateway:main',
    ]},
    tests_require=['pytest'],
)
