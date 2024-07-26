import os
import subprocess

from setuptools import find_packages, setup
from torch.utils.cpp_extension import BuildExtension, CUDAExtension, CppExtension

def make_cuda_ext(name, module, sources):
    cuda_ext = CUDAExtension(
        name='%s.%s' % (module, name),
        sources=[os.path.join(*module.split('.'), src) for src in sources]
    )
    return cuda_ext

from package_info import __version__
from package_info import __package_name__
from package_info import __homepage__
from package_info import __download_url__
from package_info import __description__
from package_info import __license__


if __name__ == '__main__':
    # Get the include directory for Python
    python_cflags = subprocess.check_output(['python3-config', '--cflags']).decode().strip()
    python_include_dir = next(opt[2:] for opt in python_cflags.split() if opt.startswith('-I'))

    setup(
        name='SVO',
        version = "0.0",
        description='SVO build in CUDA language',
        install_requires=[],
        cmdclass={
            'build_ext': BuildExtension,
        },
        ext_modules=[

            CUDAExtension('SVO_filtering', 
            sources = ['src/SVO_filtering.cu'],
            include_dirs=[python_include_dir,\
                os.path.join(os.getcwd(),"include")],
            # dlink_libraries=["../lib/build/libBgfilter.so"],
            extra_compile_args={'cxx': ['-std=c++17', '-g'],
            'nvcc':  [ "-std=c++17", "--use_fast_math", "-O3"],
            },
            ),
           
        ],
    )

    setup(name=__package_name__,
        version=__version__,
        description=__description__,
        url=__homepage__,
        download_url=__download_url__,
        license=__license__,
        packages=find_packages())
