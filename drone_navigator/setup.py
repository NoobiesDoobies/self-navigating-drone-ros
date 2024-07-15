from setuptools import setup

package_name = 'drone_navigator'
setup(
    name=package_name,
    version='1.0',
    package_dir={"": str("src")},
    packages=[package_name],

    install_requires=[
        'numpy',
        'matplotlib',
        'rclpy',
        # Add any other dependencies here
    ],
    author='Carlios Eryan',
    author_email='carlisoeryan20@gmail.com',
    description='Self-navigating drone package',
    url='https://github.com/NoobiesDoobies/self-navigating-drone-ros',

    entry_points={
        'console_scripts': [
            'path_finder = drone_navigator.path_finder:main',
        ],
    },
)