from distutils.core import setup
from setuptools import find_packages

setup(
    name='uav_classification_routing',
    version='6.9.0',
    packages=find_packages(),
    install_requires=[
                      'networkx==3.3',
                      'matplotlib==3.9.0',
                      'numpy==1.26.4',
                      'pandas==2.2.2',
                      ],
    license='Liscence to Krill',
)
