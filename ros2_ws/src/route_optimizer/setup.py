from setuptools import setup

package_name = 'route_optimizer'

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
    description='Route cost and visit order calculation.',
    license='TODO',
    tests_require=['pytest'],
)
