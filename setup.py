"""BMM350 setup"""

# Path processing
from glob import glob

# XML parsing
from xml.etree import ElementTree

# Setuptools
from setuptools import find_packages
from setuptools import setup


# Try to get package info
# Since checking everything for existence would be cumbersome, we disable strict typing here
# mypy: no-strict-optional
package = ElementTree.parse("package.xml").getroot()
PACKAGE_NAME = package.find("name").text

setup(
    name=PACKAGE_NAME,
    version=package.find("version").text,
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{PACKAGE_NAME}"]),
        (f"share/{PACKAGE_NAME}", ["resource/parameters.yaml"]),
        (f"share/{PACKAGE_NAME}", ["package.xml"]),
        (f"share/{PACKAGE_NAME}", glob("launch/*.*")),
    ],
    install_requires=["setuptools", "pytest", "pytest-cov", "smbus2"],
    tests_require=["pytest"],  # to tell colcon that we run pytest
    zip_safe=True,
    maintainer=package.find("maintainer").text,
    maintainer_email=package.find("maintainer").attrib["email"],
    description=package.find("description").text,
    license=package.find("license").text,
    entry_points={"console_scripts": ["bmm350 = bmm350.node:main"]},
)
