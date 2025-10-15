
from glob import glob
import os

from setuptools import setup

package_name= 'trimble_gnss_driver'

setup(
    name=package_name,
    version='0.0.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Michael Hutchinson',
    author_email='mhutchinson@sagarobotics.com',
    maintainer='Michael Hutchinson',
    maintainer_email='mhutchinson@sagarobotics.com',
    description='Trimble GNSS receiver GSOF driver.',
    license='See license',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gsof_driver = trimble_gnss_driver.gsof_driver:main',
        ],
    },
)
