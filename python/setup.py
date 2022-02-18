from setuptools import setup, find_packages

setup(
    name="loco_powertrain",
    version="0.1.0",
    description=
    "a custom locomotive powertrain",
    url="https://github.nrel.gov/MBAP/altrios_nrel",
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "License :: Other/Proprietary License",
        "Operating System :: OS Independent",
        "Programming Language :: Python :: 3.8",
        "Topic :: Scientific/Engineering"
    ],
    packages=find_packages(),
    python_requires=">=3.8",
    install_requires=[
        "numpy",
    ],
    author="National Renewable Energy Laboratory",
    license="Copyright ©2020 Alliance for Sustainable Energy, LLC All Rights Reserved",
)