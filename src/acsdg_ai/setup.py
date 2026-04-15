from setuptools import setup

package_name = 'acsdg_ai'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ACSDG Dev',
    maintainer_email='dev@acsdg.local',
    description='ACSDG AI inference nodes',
    license='MIT',
    entry_points={},
)
