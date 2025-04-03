from setuptools import setup

package_name = 'robot_hospital_logistics'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ton Nom',
    maintainer_email='ton.email@example.com',
    description='Un robot hospitalier pour la gestion logistique',
    license='Propriétaire',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'auto_explore = robot_hospital_logistics.nodes.auto_explore:main',  # Correction ici
            'qr_code_detector = robot_hospital_logistics.qr_code_detector:main',
        ],
    },
)
