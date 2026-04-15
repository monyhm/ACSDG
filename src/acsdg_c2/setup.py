from setuptools import setup

package_name = 'acsdg_c2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mal',
    maintainer_email='mal@acsdg.local',
    description='C2 nodes for the ACSDG system',
    license='Apache-2.0',
)
