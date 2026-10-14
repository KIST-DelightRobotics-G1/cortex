from setuptools import find_packages, setup

package_name = 'cortex_common'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='박성용',
    maintainer_email='park50260@gmail.com',
    description='Helpers shared by cortex nodes (opt-in events) and the session recorder.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'session_recorder_node = cortex_common.session_recorder_node:main',
        ],
    },
)
