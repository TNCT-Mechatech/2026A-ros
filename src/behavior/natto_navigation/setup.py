from setuptools import setup

setup(
    name='natto_navigation',
    version='1.0.0',
    packages=['natto_navigation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/natto_navigation']),
        ('share/natto_navigation', ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    maintainer='Kazusa Hashimoto',
    maintainer_email='kzs321kzs@gmail.com',
    description='Button-selected destinations with cancellable navigation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={'console_scripts': [
        'joy_goal_selector = natto_navigation.joy_goal_selector:main',
        'navigation_manager = natto_navigation.navigation_manager:main',
    ]},
)
