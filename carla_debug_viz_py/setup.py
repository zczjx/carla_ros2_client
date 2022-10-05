from setuptools import setup

package_name = 'carla_debug_viz_py'

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
    maintainer='clarencez',
    maintainer_email='zhou_chenz@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_debug_viz = carla_debug_viz_py.carla_debug_viz:main'
        ],
    },
)
