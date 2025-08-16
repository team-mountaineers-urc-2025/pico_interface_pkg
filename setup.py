from setuptools import find_packages, setup

package_name = 'pico_interface_pkg'

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
    maintainer='Sam',
    maintainer_email='moodywvs@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pico_host_node = pico_interface_pkg.host_node:main',
            'pico_led_node = pico_interface_pkg.temp_node:main',
            'pico_potentiometer_node = pico_interface_pkg.potentiometer_node:main',
            'pico_arm_node = pico_interface_pkg.pico_comm_claw:main',
            'pico_science_node = pico_interface_pkg.pico_science_node:main'
        ],
    },
)
