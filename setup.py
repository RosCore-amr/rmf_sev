from setuptools import find_packages, setup

package_name = "rmf_sev"

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
    maintainer="mm",
    maintainer_email="engineer.pqm@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "transfer_if = rmf_sev.transfer_infor:main",
            "controlsystem = rmf_sev.control_system:main",
        ],
    },
)
