from setuptools import find_packages, setup

package_name = 'nebula_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alitalha',
    maintainer_email='alitqlhq@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'image_processed_subscriber = nebula_test.image_processed_subscriber:main',
        'image_raw_subscriber = nebula_test.image_raw_subscriber:main'
        ],
    },
)
