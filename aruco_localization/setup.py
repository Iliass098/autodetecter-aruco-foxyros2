# ~/aruco./aruco_localization/setup.py
from setuptools import setup, find_packages

package_name = 'aruco_localization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # ✅ Automatically finds 'aruco_localization'
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ArUco localization package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_localization.aruco_detector:main',
        ],
    },
)
