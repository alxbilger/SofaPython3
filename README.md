# SofaPython3 plugin

[![Documentation](https://img.shields.io/badge/doc-on_website-green.svg)](https://sofapython3.readthedocs.io/en/latest/index.html)
[![Discord](https://img.shields.io/badge/chat-on_Discord-darkred.svg)](https://discord.gg/muqNzQpv)
[![Support](https://img.shields.io/badge/support-on_GitHub_Discussions-blue.svg)](https://github.com/sofa-framework/sofa/discussions/categories/sofapython3)

This project is composed of a Sofa plugin to embed a python interpreter into a Sofa based simulation as well as several python modules that exposes the different c++ components used in Sofa. The binding is designed to be idiomatic python3 API with tight integration for numpy. This project is in a WIP state, please use it only if you are willing to help in the developement. 

## Installation 

### Requirement Install
- pybind11 (minimal 2.3)
- cmake (minimal 3.11)
- developement package for python3 (python3-dev)

### In-tree build
Add this directory path in `CMAKE_EXTERNAL_DIRECTORIES`.

NB: This plugin cannot be build through in-build process when the old SofaPython plugin is activated. To have both SofaPython3 and SofaPython you need to use out-of-tree build. 

### Out-of-tree build
This plugin should compile with out-of-tree builds.
You might need to add the Sofa's installation path to the CMake prefix path. If you compiled Sofa in directory _$SOFA_ROOT/build_, consider doing an install step (make install, ninja install, etc.) and adding this installation path (example `cmake -DCMAKE_PREFIX_PATH=$SOFA_ROOT/build/install/lib/cmake ..`).

### Changing the python path
The compilation of SofaPython3 plugin and bindings are tied to the python core library found during the CMake stage.
To change the python version used for the compilation, you can either:
1. Provide the python executable path with `Python_EXECUTABLE`
 ```cmake -DPython_EXECUTABLE=/usr/local/bin/python3 ..```
2. Provide the python root path with `Python_ROOT_DIR`
 ```cmake -DPython_ROOT_DIR=/usr/local ..```

To see all the hints that can be provided to CMake, see the official CMake documentation on Python :
https://cmake.org/cmake/help/latest/module/FindPython.html

### Installing the python 3 bindings
In order to use SofaPython3 bindings directly into a python3 interpreter, Python needs to find the bindings libraries. 
This can be done automatically by going into the build directory of SofaPython3, and starting the cmake installation 
command:

```
plugin.SofaPython3/build $ cmake --install . 
```

This will first install the SofaPython3 plugin and bindings into the `install` directory, and then create symbolic links
to the installed bindings into the python's user site-package directory (the directory returned by 
```python3 -m site --user-site```). After that, you should be able to import the SofaPython3 bindings directly into
python:

```python
$ python3
>>> import SofaRuntime
>>> import Sofa
>>> root = Sofa.Core.Node("root")
```

You can change the directory where the links are created by setting the cmake variable 
```SP3_PYTHON_PACKAGES_LINK_DIRECTORY```. For example, the following will create symbolic links into the 
```/usr/lib/python3.8/dist-packages```, hence making the SofaPython3 bindings available to python3 for all the system
users.

```
plugin.SofaPython3/build $ cmake -DSP3_PYTHON_PACKAGES_LINK_DIRECTORY=/usr/lib/python3.8/dist-packages .
plugin.SofaPython3/build $ cmake --install . 
```

Finally, you can disable the automatic link creation with the cmake option ```SP3_LINK_TO_USER_SITE```:
```
plugin.SofaPython3/build $ cmake -DSP3_LINK_TO_USER_SITE=OFF .
```

## Features

### The Sofa python module:
Expose the base Sofa object to make a scene. 
- binding of BaseObject, BaseNode, Base, BaseData [DONE] 
- copy-less API to access the sofa Data containers [WIP] 
- implement custom sofa object (ForceField,  Controller) in python [POC]
- docstring with sphinx content [WIP]

Try it: ```python import Sofa```

### The SofaRuntime python module:

- access the runtime specific stuff (GUI, GLViewers, runSofa internal status) [POC]
- docstring with sphinx content [TBD]

Try it: ```python import SofaRuntime```

### Developer's environment

- autogenerated documentation using sphinx [DONE]
- automated update the docs from the c++'s docstring: https://sofapython3.readthedocs.io/en/latest/ [WIP]
- code completion with common editor [WIP, some editor are not working with c++ modules]


### Execution environment: 

- SofaPython3 is a plugin to include a python3 environment in a Sofa scene [DONE],

Try it: ```xml <RequiredPlugin='SofaPython3'/>```


- Sofa and SofaRuntime are the python module that can be imported in any python interpreter (python3, ipython, jupyter)[DONE], 

Try it: ```python python3 minimalscene.py```

- Access to Sofa simulation within the MathLab python interpreter [WIP-POC].  
- Make a full python GUI application (with UI framework like PySide2, pygame) and render an integrated sofa scene in an opengl context [POC]
 

## Development history:
### June 19, 2019
- documentation extraction from .cpp (Thierry)
- refactoring the modules & SofaRuntime (Jean Nicolas)
- Data access & code cleaning (Bruno)
```python3
c1 = root.addChild("child1")
c2 = root.addChdil("child2")
o1 = root.addObject("MechanicalObject", dofs)
p = root.child1.child2.dofs.position         ## Slow acces to data 
p = root["child1.child2.dofs.position"]      ## Ffast access
```
- improve onEvent() method in Controller (Bruno)
- autodoc & docstring generation on https://sofapython3.readthedocs.io/en/latest/ (Damien)


