from setuptools import setup, find_packages

package_name = 'camera_utils'

with open('README.md', 'r') as file:
    long_description = file.read()

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[],
    install_requires=['setuptools', 'numpy', 'open3d', 'pyrealsense2', 'imutils'],
    zip_safe=True,
    url='https://github.com/IASRobolab/camera_utils',
    maintainer='frollo',
    maintainer_email='rollo.f96@gmail.com',
    description='This package contains functions useful for camera use and other computer vision computations',
    license='GNU GENERAL PUBLIC LICENSE v3',
    # entry_points={},
    # extras_require = { 'dev': ['pytest>=3.7']}, # to install with this run 'pip install -e .[dev]
    long_description=long_description,
    long_description_content_type='text/markdown',
)
