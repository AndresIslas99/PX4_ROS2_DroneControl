from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'web_gcs'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include templates
        (os.path.join('share', package_name, 'templates'),
            glob('web_gcs/templates/*.html') if glob('web_gcs/templates/*.html') else []),
        # Include static files
        (os.path.join('share', package_name, 'static', 'css'),
            glob('web_gcs/static/css/*') if glob('web_gcs/static/css/*') else []),
        (os.path.join('share', package_name, 'static', 'js'),
            glob('web_gcs/static/js/*') if glob('web_gcs/static/js/*') else []),
    ],
    package_data={
        'web_gcs': [
            'templates/*.html',
            'static/css/*',
            'static/js/*',
        ],
    },
    include_package_data=True,
    install_requires=[
        'setuptools',
        'flask>=2.3.0',
        'flask-socketio>=5.3.0',
        'flask-cors>=4.0.0',
        'python-socketio>=5.9.0',
        'eventlet>=0.33.0',
    ],
    zip_safe=True,
    maintainer='andres',
    maintainer_email='andresislas99@outlook.com',
    description='Professional Web-based Ground Control Station for PX4',
    license='Apache-2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'web_gcs = web_gcs.gcs_server:main',
        ],
    },
)
