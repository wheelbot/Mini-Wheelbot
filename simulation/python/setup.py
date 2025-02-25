from distutils.core import setup

setup(name='wheelbot_dynamics',
   version='0.1',
   python_requires='>=3.8',
   description='A python packages with ODEs for the Wheelbot',
   author='Henrik Hose',
#    use_scm_version={
#      "fallback_version": "0.1-local",
#      "root": "../..",
#      "relative_to": __file__
#    },
   license='MIT',
   include_package_data = True,
   setup_requires=['setuptools_scm'],
   install_requires=[
      'numpy',
      'matplotlib',
      'casadi>=3.6.0',
      'acados_template',
   ],
   py_modules=["wheelbot_dynamics"]
)