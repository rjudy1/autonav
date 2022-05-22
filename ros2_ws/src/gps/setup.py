from setuptools import setup

package_name = 'gps'

setup(
    name=package_name,
    version='2.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pynmea2'],
    zip_safe=True,
    maintainer='autonav',
    maintainer_email='rljudy4981@icloud.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "gps_publisher=gps.gps_reader:main"
        ],
    },
)
