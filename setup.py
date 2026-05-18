from setuptools import find_namespace_packages, setup


setup(
    name="orca_core",
    version="0.0.0",
    description="ORBIT Teleop ORCA core utilities and drivers.",
    packages=find_namespace_packages(
        include=["orca_core*", "helios_core*", "hardware*"],
        exclude=["*.tests", "*.tests.*"],
    ),
    # Dependency ownership lives in the parent ORBIT_Teleop environment files.
    # Keep this submodule non-standalone so Jetson/offboard deps stay explicit.
    install_requires=[],
    include_package_data=True,
)
