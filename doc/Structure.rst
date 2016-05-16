=====================
Structure of the code
===================== 



blender2sofa module (\_\_init\_\_.py)
=====================================
.. module:: blender2sofa

Stores the metadata about the plugin. Contains routines for 
registration/unregistartion of all the other modules.


blender2sofa.export
===================
.. module:: blender2sofa.export

Routines that facilitate generation of XML (or Lua) from a Blender scene.
The conversion consists of many tasks that are all contained in this module:
 * Conversion of arbitrary geometry to triangular and quadrilateral surfaces
 * Auto-generation of volumetric geometry from curves or surfaces in case of thick-shell and thick-curve
 * Splitting the output into many files when enabled
 * Generation of proper XML tags that represent the geometry and topology based on the template
 * Intelligent calculation of some parameters based on the given geometry
 * Emitting predefined hierarchies of XML tags to facilitate certain behavior in SOFA (e.g. changing instruments)

The entry point is the ExportToSOFA operator. Which in turn calls the exportScene function with
the proper user specified options. exportScene calls many export[XXXX] functions based on the templates
associated with the objects.

There are also many support routines for common tasks; following the DRY principle.

.. class:: ExportTOSOFA


blender2sofa.ui
===============
.. module:: blender2sofa.ui

Definition of the user-interface as the side-panel in the 3D view.

blender2sofa.conn_tiss
======================
.. module:: blender2sofa.conn_tiss
connective tissue generation operator


blender2sofa.io_mesh
====================
.. module:: blender2sofa.io_msh
The self-contained plug-in for import/export of GMSH files. Included here for convenience it can be used by itself as a plugin


blender2sofa.lua_format
=======================
.. module:: blender2sofa.lua_format

This file contains serialization routines that allow generating
Lua code that creates a SOFA scene from a XML tree data structure.

The generated Lua code uses SaLua bindings to SOFA to create a
hierarchy of nodes and objects similar to what the SOFA XML reader
would create given the XML input.

Instead of creating special data structures for the description of the
scene in Python code, we utilize ElementTree data structures since and
just provide the serialization routines. ElementTree has some limitations;
but it performs perfectly for this purpose.

