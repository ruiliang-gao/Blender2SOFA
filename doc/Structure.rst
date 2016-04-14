=====================
Structure of the code
==================== 

* \_\_init\_\_.py: registration of all the submodules, plugin meta-data information
* export.py: routines for exporting the scene into XML or Lua
* lua\_format.py: Routines for exporting the scene hierarchy (ElementTree) into Lua format
* ui.py: Definition of the user-interface as the side-panel in the 3D view.
* conn\_tiss.py: connective tissue generation operator
* io\_msh.py: The self-contained plug-in for import/export of GMSH files. Included here for convenience it can be used by itself as a plugin
