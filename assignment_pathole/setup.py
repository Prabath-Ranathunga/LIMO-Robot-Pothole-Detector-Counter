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
            "pothole_detector = assignment_pathole.pothole_detector:main",
            "pothole_detector_real = assignment_pathole.pothole_detector_real:main",
            "pothole_counter = assignment_poahole.pothole_counter:main"
        ],
    },
)
