import setuptools

setuptools.setup(
    name="urdf-kit",
    version="0.4",
    author="Lai Kan Muk",
    author_email="lkmuk2017@gmail.com",
    description=""" Useful addons for 
    (1) URDF editing, particularly conceived for postprocessing with Onshape-to-Robot. 
    (2) inspecting/ modifying the kinematics topology 
    (3) extracting multibody parameters
    """,
    python_requires='>=3.9',
    packages=setuptools.find_packages(),
    install_requires=[
        "numpy",
        "spatialmath-python>=1.0.3",
        "pyyaml>=6",
    #     "onshape-to-robot=0.3.17" # I guess it's not a strict requirement
    ],
)
