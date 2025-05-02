from setuptools import setup
from setuptools import find_packages

package_name = 'camera_servo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=[]),
    # If your Python node is named `camera_servo_node.py` inside `camera_servo/camera_servo/`,
    # then the module path is `camera_servo.camera_servo_node:main`
    entry_points={
        'console_scripts': [
            'camera_servo = camera_servo.camera_servo_node:main',
        ],
    },
)
