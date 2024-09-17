from setuptools import find_packages, setup
import os

package_name = 'planiranje_putanje'

data_files=[
        ('share/' + package_name, ['package.xml'])
    ]

def package_files(data_files, directory_list):

    paths_dict = {}

    for directory in directory_list:
        
        for (path, directories, filenames) in os.walk(directory):

            for filename in filenames:

                file_path = os.path.join(path, filename)
                install_path = os.path.join('share', package_name, path)
                
                if install_path in paths_dict.keys():
                    paths_dict[install_path].append(file_path)   
                else:
                    paths_dict[install_path] = [file_path]
                
    for key in paths_dict.keys():
        data_files.append((key, paths_dict[key]))

    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=package_files(data_files, ['launch/', 'rviz/']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kresimirhartl',
    maintainer_email='kh239762@fsb.hr',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hedgehog_to_pose = planiranje_putanje.hedgehog_to_pose:main',
            'prikaz_polja = planiranje_putanje.prikaz_polja:main',
            'odom_to_tf = planiranje_putanje.odom_to_tf:main',
            'navigacija_mpp = planiranje_putanje.navigacija_mpp:main'
        ],
    },
)
