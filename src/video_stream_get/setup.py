from setuptools import find_packages, setup

package_name = 'video_stream_get'

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
    maintainer='kijung914',
    maintainer_email='kijung914@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'streamer_node = video_stream_get.ros_streamer:main',
            'collector_node = video_stream_get.ros_image_collector:main'
        ],
    },
)
