from setuptools import setup, Extension
from Cython.Build import cythonize
import numpy

extensions = [
    Extension(
        "compute_overlap",
        sources=["compute_overlap.pyx"],
        include_dirs=[numpy.get_include()]
    )
]

setup(
    ext_modules=cythonize(extensions),
)
