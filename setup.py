import setuptools

setuptools.setup(
    name="urdf-kit",
    version="0.1",
    author="Lai Kan Muk",
    author_email="lkmuk2017@gmail.com",
    description="Useful addons for URDF editing, particularly conceived for postprocessing with Onshape-to-Robot's URDF export.",
    python_requires='>=3.9',
    packages=setuptools.find_packages(),
    # install_requires=[
    #     "onshape-to-robot=0.3.17" # I guess it's not a straight requirements
    # ],
)
