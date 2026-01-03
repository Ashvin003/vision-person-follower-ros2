from setuptools import setup

package_name = 'vision_person_detector'

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
    maintainer='ashvin',
    maintainer_email='ashvin@todo.todo',
    description='YOLOv8 person detector and follower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # PERSON DETECTOR
            'person_detector = vision_person_detector.person_detector:main',

            # FOLLOWER NODE
            'simple_follower = vision_person_detector.follower_node:main',
        ],
    },
)
