from setuptools import find_packages, setup

package_name = 'manus_to_inspire'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='brian clark',
    maintainer_email='brian@dwell.bot',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inspire = manus_to_inspire.manus_ik:main'
        ],
    },
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/inspire.launch.py']),  # Include your launch files here
    ],
)
