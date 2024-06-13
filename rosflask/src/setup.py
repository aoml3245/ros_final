from setuptools import setup

package_name = 'my_python_pkg'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 'talker = my_python_pkg.talker:main',
            # 'flask_server = my_python_pkg.flask_server:main',
            # 'message_printer = my_python_pkg.message_printer:main',
            'dual_node_runner = my_python_pkg.dual_node_runner:main',
        ],
    },
)
