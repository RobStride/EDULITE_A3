from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'el_a3_web_ui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.py')),
        # Templates
        (os.path.join('share', package_name, 'templates'), 
            glob('templates/*.html')),
        # Static files - CSS
        (os.path.join('share', package_name, 'static', 'css'), 
            glob('static/css/*.css')),
        # Static files - JS
        (os.path.join('share', package_name, 'static', 'js'), 
            glob('static/js/*.js')),
        # Static files - URDF
        (os.path.join('share', package_name, 'static', 'urdf'), 
            glob('static/urdf/*.urdf')),
        # Static files - meshes
        (os.path.join('share', package_name, 'static', 'urdf', 'meshes'), 
            glob('static/urdf/meshes/*.stl')),
    ],
    install_requires=[
        'setuptools',
        'flask>=2.0',
        'flask-socketio>=5.0',
        'python-socketio>=5.0',
        'eventlet>=0.33',
    ],
    zip_safe=True,
    maintainer='EL-A3 Developer',
    maintainer_email='user@example.com',
    description='EL-A3 Web-based Robot Control Interface',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web_server = el_a3_web_ui.web_server:main',
        ],
    },
)
