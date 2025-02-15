from setuptools import setup, find_packages

pkg_name = 'turtle_bot_3'

setup(
    name= pkg_name,
    version='0.0.0',
    packages= [pkg_name], #find_packages()  # Busca paquetes en la raíz - Quitar Entry Points
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + pkg_name]),
        ('share/'+ pkg_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            "turtle_bot_teleop = turtle_bot_3.turtle_bot_teleop:main",
            "turtle_bot_interface = turtle_bot_3.turtle_bot_interface:main",
            'turtle_bot_player = turtle_bot_3.turtle_bot_player:main',
        ],
    },
)
