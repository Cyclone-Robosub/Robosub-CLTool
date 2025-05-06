from setuptools import find_packages, setup

package_name = "pwm_cltool"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Cyclone Robosub",
    maintainer_email="crs.ucd@gmail.com",
    description="TODO: Package description",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pwm_cltool_node = pwm_cltool.pwm_cltool:main",
        ],
    },
)
