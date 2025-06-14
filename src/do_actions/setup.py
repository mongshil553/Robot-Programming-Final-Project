from setuptools import find_packages, setup

package_name = 'do_actions'

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
            'action_sim = do_actions.action_server_sim:main',
            'action_servers = do_actions.action_server:main',
            'action_clients = do_actions.action_client:main',
            'servo_node = do_actions.servo:main'
        ],
    },
)
