import sys

try:
    from skbuild import setup
except ImportError:
    print(
        "Please update pip, you need pip 10 or greater,\n"
        " or you need to install the PEP 518 requirements in pyproject.toml yourself",
        file=sys.stderr,
    )
    raise

from setuptools import find_packages

setup(
    name="altrios_py",
    version="0.0.1",
    description="a minimal example package (with pybind11)",
    author="Henry Schreiner",
    license="MIT",
    packages=find_packages(where="source"),
    package_dir={"": "source"},
    cmake_install_dir="source/altrios_py",
    include_package_data=True,
    install_requires=["numpy", "pandas"],
    extras_require={"test": ["pytest"]},
    python_requires=">=3.6",
)
