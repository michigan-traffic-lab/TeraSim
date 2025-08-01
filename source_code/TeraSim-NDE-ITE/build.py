from setuptools import setup, find_packages
from setuptools.extension import Extension
from Cython.Build import cythonize
import numpy

extensions = [
    Extension(
        "terasim_nde_nade.vehicle.nde_vehicle_utils_cython",
        ["terasim_nde_nade/vehicle/nde_vehicle_utils_cython.pyx"],
        include_dirs=[numpy.get_include()],
    ),
]

setup(ext_modules=cythonize(extensions), script_args=["build_ext", "--inplace"])
