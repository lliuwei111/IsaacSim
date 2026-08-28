import os
from setuptools import setup, __version__ as setuptools_version
from packaging.version import Version
from glob import glob

package_name = 'pro450_isaacsim'

# Checking the setuptools version
use_dash_separated_options = Version(setuptools_version) < Version("58.0.0")


# Dynamically generate setup.cfg content
setup_cfg_content = """
[develop]
{script_option}=$base/lib/{package_name}

[install]
{install_scripts_option}=$base/lib/{package_name}
""".format(
    package_name=package_name,
    script_option='script-dir' if use_dash_separated_options else 'script_dir',
    install_scripts_option='install-scripts' if use_dash_separated_options else 'install_scripts'
)

# Write the contents to setup.cfg
with open("setup.cfg", "w") as f:
    f.write(setup_cfg_content)

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
    maintainer='u2',
    maintainer_email='u2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'follow_display = pro450_isaacsim.follow_display:main',
            'slider_control = pro450_isaacsim.slider_control:main',
            'slider_control1 = pro450_isaacsim.slider_control1:main',
            'pick_and_place_menu = pro450_isaacsim.pick_and_place_menu:main',
            'pick_and_place_maduo = pro450_isaacsim.pick_and_place_maduo:main',
            'pick_and_place_fenjian = pro450_isaacsim.pick_and_place_fenjian:main',
        ],
    },
)
