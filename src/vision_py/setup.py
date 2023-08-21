from setuptools import setup

package_name = 'vision_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yangqi',
    maintainer_email='yangqi@cau.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "subscribed_img_viewer = vision_py.subscribed_img_viewer:main",
            "test_publisher = vision_py.test_publisher:main",
            "tgt2cam = vision_py.tgt2cam:main",
            "calib_eyeinhand = vision_py.calib_eyeinhand:main"
        ],
    },
)
