from setuptools import find_packages, setup

package_name = 'assignment_pathole'

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
    maintainer='cpranathunga',
    maintainer_email='c.p.ranathunga@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "mover = assignment_pathole.mover:main",
            "pathole_detector = assignment_pathole.pathole_detector:main",
            "pathole_counter = assignment_pathole.pathole_counter:main"
        ],
    },
)
