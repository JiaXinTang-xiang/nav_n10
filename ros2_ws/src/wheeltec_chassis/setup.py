import os
from glob import glob
from setuptools import setup
from setuptools.command.install import install

package_name = 'wheeltec_chassis'


class CustomInstall(install):
    """安装后创建 lib/<pkg>/ 软链接，兼容 ros2 run"""
    def run(self):
        install.run(self)
        lib_dir = os.path.join(self.install_base, 'lib', package_name)
        bin_dir = os.path.join(self.install_base, 'bin')
        if os.path.isdir(bin_dir):
            os.makedirs(lib_dir, exist_ok=True)
            target = os.path.relpath(os.path.join(bin_dir, 'chassis_bridge'), lib_dir)
            link_path = os.path.join(lib_dir, 'chassis_bridge')
            if not os.path.exists(link_path):
                os.symlink(target, link_path)


setup(
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    cmdclass={'install': CustomInstall},
    zip_safe=True,
)
