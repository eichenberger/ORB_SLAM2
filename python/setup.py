#!/usr/bin/env python
"""
setup.py file for SWIG example
"""
from distutils.core import setup, Extension
import os

example_module = Extension('_orbslam',
                           sources=['orbslam_wrap.cxx'],
                           libraries = ['ORB_SLAM2', 'opencv_core'],
                           library_dirs = ["../lib"],
                           include_dirs = ['..'],
                           )
setup (name = 'orbslam',
       version = '0.1',
       author = "SWIG Docs",
       description = """Simple swig example from docs""",
       ext_modules = [example_module],
       py_modules = ["example"],)
