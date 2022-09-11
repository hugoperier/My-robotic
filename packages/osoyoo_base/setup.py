from setuptools import setup

package_name = 'osoyoo_base'

setup(
    name=package_name,
    version='0.0.1',
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
    description='Raspberry pi robotic base',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'osoyoo_base = osoyoo_base.osoyoo_base:main'
        ],
    },
)
