from setuptools import find_packages, setup

package_name = 'pilot_gui'

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
    maintainer='jhsrobo',
    maintainer_email='jammerand14@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_cameras = pilot_gui.find_cameras:main',
            'switch_cameras = pilot_gui.switch_cameras:main',
        ],
    },
)
