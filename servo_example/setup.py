from setuptools import find_packages, setup

package_name = 'servo_example'

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
    maintainer='h6x_arai',
    maintainer_email='ray255ar@gmail.com',
    description='servo example',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'servo_example_service = servo_example.servo_example_service:main',
            'servo_example_action = servo_example.servo_example_action:main'
        ],
    },
)
