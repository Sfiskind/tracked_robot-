from setuptools import setup

package_name = 'your_twist_converter'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Optionally, add your launch files if needed:
        # ('share/' + package_name + '/launch', ['launch/your_launch_file.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Your Name',
    author_email='your.email@example.com',
    description='A package that converts Twist to TwistStamped.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_converter = your_twist_converter.twist_converter:main'
        ],
    },
)