from setuptools import find_packages, setup

package_name = 'pc4'

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
    maintainer='gwak',
    maintainer_email='gwak@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'AMR2_control = pc4.AMR2_control:main',
            'admin_monitor_pc4 = pc4.admin_monitor_pc4:main'

        ],
    },
)
