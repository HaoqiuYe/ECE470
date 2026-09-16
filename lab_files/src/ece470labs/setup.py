from setuptools import find_packages, setup

package_name = 'ece470labs'

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
    maintainer='d-block',
    maintainer_email='d-block@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'lab2_exec = ece470labs.lab2_exec:main',            
            'lab3_exec = ece470labs.lab3_exec:main',            
            'lab4_exec = ece470labs.lab4_exec:main',
            'lab5_exec = ece470labs.lab5_exec:main',            

        ],
    },
)
