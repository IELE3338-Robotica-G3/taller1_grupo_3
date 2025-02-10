from setuptools import setup, find_packages

setup(
    name='turtle_bot_3',
    version='0.0.0',
    packages=find_packages(),  # Busca paquetes en la raíz
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/turtle_bot_3']),
        ('share/turtle_bot_3', ['package.xml']),
    ],
    install_requires=['setuptools'],
    entry_points={
        'console_scripts': [
            'teleop = turtle_bot_3/teleop:main', 
            'player = turtle_bot_3/teleop:main'
        ],
    },
)
