from setuptools import setup, find_packages

setup(
    name="tangods_pic863mercury",
    version="0.0.1",
    description="PIC863Mercury Tango Device Server",
    author="Martin Hennecke",
    author_email="hennecke@mbi-berlin.de",
    python_requires=">=3.6",
    entry_points={"console_scripts": ["PIC863Mercury = tangods_pic863mercury:main"]},
    license="MIT",
    packages=["tangods_pic863mercury"],
    install_requires=["pytango", "pyserial"],
    url="https://github.com/MBI-Div-b/pytango-PIC863Mercury",
    keywords=[
        "tango device",
        "tango",
        "pytango",
        "pic863mercury",
    ],
)
