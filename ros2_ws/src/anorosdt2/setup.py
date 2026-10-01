import os
from glob import glob
from setuptools import setup
from setuptools.command.install import install

package_name = 'anorosdt2'


class CustomInstall(install):
    """安装后创建 lib/anorosdt2/anoros_dt 软链接，兼容 ros2 run"""
    def run(self):
        install.run(self)
        # 在 install 前缀下创建 lib/<package_name>/ 指向 bin/<executable> 的软链接
        lib_dir = os.path.join(self.install_base, 'lib', package_name)
        bin_dir = os.path.join(self.install_base, 'bin')
        if os.path.isdir(bin_dir):
            os.makedirs(lib_dir, exist_ok=True)
            link_path = os.path.join(lib_dir, 'anoros_dt')
            target = os.path.relpath(os.path.join(bin_dir, 'anoros_dt'), lib_dir)
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
