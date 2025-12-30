from setuptools import find_packages, setup

package_name = 'remote_control_pkg'

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
    maintainer='lianxin',
    maintainer_email='218019067@link.cuhk.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'keyboard_control_node = remote_control_pkg.keyboard_control_node:main',
            'widowx250s_ee_contorl_node = remote_control_pkg.widowx250s_ee_contorl_node:main',
            'gello_control_node = remote_control_pkg.gello_control_node:main',

        ],
    },
)
