from setuptools import setup

package_name = 'camera'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hugoperier',
    maintainer_email='hugo.perier@protonmail.com',
    description='Raspberry Pi camera node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rpi_camera = camera.camera:main'
        ],
    },
)
