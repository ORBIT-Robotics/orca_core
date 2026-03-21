from setuptools import find_namespace_packages, setup


setup(
    name="orca_core",
    version="0.0.0",
    description="ORBIT Teleop ORCA core utilities and drivers.",
    packages=find_namespace_packages(
        include=["orca_core*", "helios_core*", "hardware*"],
        exclude=["*.tests", "*.tests.*"],
    ),
    install_requires=[],
    include_package_data=True,
)
