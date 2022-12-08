import shutil
import subprocess
import os
import glob
import shutil

from setuptools import setup, Extension
from setuptools.command.build_ext import build_ext

__library_file__ = './lib/g2o*.so'
__version__ = '0.0.1'

class CMakeExtension(Extension):
    def __init__(self, name, cmake_lists_dir='.', **kwa):
        Extension.__init__(self, name, sources=[], **kwa)
        self.cmake_lists_dir = os.path.abspath(cmake_lists_dir)

class CMakeBuildExtension(build_ext):
    def build_extensions(self):
        # Ensure that CMake is present and working
        try:
            subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError('Cannot find CMake executable')

        for ext in self.extensions:
            extdir = os.path.abspath(os.path.dirname(self.get_ext_fullpath(ext.name)))
            if not os.path.exists(self.build_temp):
                os.makedirs(self.build_temp)
            print(self.build_temp, ext.cmake_lists_dir)
            subprocess.check_call(['cmake', ext.cmake_lists_dir], cwd=self.build_temp)
            subprocess.check_call(['make'], cwd=self.build_temp)

            lib_file = glob.glob(__library_file__)
            assert len(lib_file) == 1

            shutil.copy(lib_file[0], extdir)

setup(
    name='g2opy',
    version=__version__,
    description='Python binding of C++ graph optimization framework g2o.',
    url='https://github.com/uoip/g2opy',
    license='BSD',
    ext_modules = [CMakeExtension("g2o")],
    cmdclass = {'build_ext': CMakeBuildExtension},
    py_modules=["g2o"],
    keywords='g2o, SLAM, BA, ICP, optimization, python, binding',
    long_description="""This is a Python binding for c++ library g2o
        (https://github.com/RainerKuemmerle/g2o).

        g2o is an open-source C++ framework for optimizing graph-based nonlinear
        error functions. g2o has been designed to be easily extensible to a wide
        range of problems and a new problem typically can be specified in a few
        lines of code. The current implementation provides solutions to several
        variants of SLAM and BA."""
)
