from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'learning_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]), ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tomdy.net@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'static_tf_broadcaster = learning_tf.static_tf_broadcaster:main',
            'turtle_tf_broadcaster = learning_tf.turtle_tf_broadcaster:main',
            'tf_listener = learning_tf.tf_listener:main',
            'turtle_following = learning_tf.turtle_following:main',
        ],
    },
)
