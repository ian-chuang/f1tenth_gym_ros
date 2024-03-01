from setuptools import setup

package_name = 'emergency_braking'

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
    maintainer='Yoon',
    maintainer_email='taeyoonkim1995@gmail.com',
    description='F1tenth Emergency Braking (AEB) at UCD',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'emergency_braking = emergency_braking.emergency_braking:main'
        ],
    },
)
