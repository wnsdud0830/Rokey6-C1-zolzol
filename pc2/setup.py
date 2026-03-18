from setuptools import find_packages, setup

# 추가 부분
import os
from glob import glob

package_name = 'pc2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('lib', package_name, 'templates'), glob('pc2/templates/*.html')),
        (os.path.join('lib', package_name), ['pc2/detection.pt', 'pc2/mart_1.db'])
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
            'customer_monitor_pc2 = pc2.customer_monitor:main'
        ],
    },
)
