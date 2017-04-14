from distutils.core import setup
from distutils.extension import Extension
from Cython.Distutils import build_ext

ext_modules = [Extension("autoencoder_api_cdef",
                     ["autoencoder_api_cdef.pyx"],
                     language='c++',
                     )]

setup(
  name = 'autoencoder_api_cdef',
  cmdclass = {'build_ext': build_ext},
  ext_modules = ext_modules
)