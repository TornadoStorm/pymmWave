# setup.py
from codecs import open
from os import path

from setuptools import setup  # type: ignore

__author__ = "pfeghali"

long_description: str
with open(
    path.join(path.abspath(path.dirname(__file__)), "README.md"), encoding="utf-8"
) as f:
    long_description = f.read()


setup(
    name="pymmWave",
    version="1.2.2",
    py_modules=["pymmWave"],
    description="A TI mmWave RADAR EVM integration library.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://pymmwave.readthedocs.io/en/latest/#",
    author_email="julianduehnen@gmail.com",
    include_package_data=True,
    python_requires=">3.9",
    license_files=("LICENSE"),
)
