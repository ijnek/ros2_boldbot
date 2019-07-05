from setuptools import setup

package_name = 'behavior'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    zip_safe=False,
    py_modules=[
        'look_at_ball',
        'walk_straight'
    ],
    data_files=[],
    install_requires=['setuptools'],
    maintainer='Marcus M. Scheunemann',
    maintainer_email='marcus@mms.ai',
    keywords=['ROS'],
    description='RC19 winning behavior',
    license='Apache License, Version 2.0',
    entry_points={
        'console_scripts': [
            'walk_straight = walk_straight:main',
            'look_at_ball = look_at_ball:main'
        ],
    },
)
