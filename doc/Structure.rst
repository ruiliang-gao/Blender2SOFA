=====================
Structure of the code
=====================



blender2sofa module (\_\_init\_\_.py)
=====================================
.. module:: blender2sofa

Stores the metadata about the plugin. Contains routines for
registration/unregistartion of all the other modules.

.. function:: register()

   Function that is called by Blender when the plug-in is loaded. This function
   is responsible for registering all the Operators and adding menu items and
   other UI enhancements.

.. function:: unregister()

   Called when the plug-in is unloaded. It is supposed to remove all the UI enhancements and
   unregister all the operators.


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

The entry point is the `ExportToSOFA` operator. Which in turn calls the exportScene function with
the proper user specified options. exportScene calls many `export[XXXX]` functions based on the templates
associated with the objects.

There are also many support routines for common tasks; following the DRY principle.

.. class:: ExportToSofa

   An operator registered in Blender as "Export To SOFA" when searched from the list of
   operators.

   .. method:: execute(self, context)

      This method is called after the user selects a file. This method
      calls `exportScene` to convert the scene into an `ElementTree` data structure
      and then calls `writeNodesToFile` to write it to disk

   .. attribute:: filename_ext
   .. attribute:: filter_glob
   .. attribute:: export_separate
   .. attribute:: use_selection
   .. attribute:: isolate_geometry


.. function:: writeNodesToFile(root, filepath, opt)

   Write ElementTree data structure (`root`) to the specified file. Based on the
   selected filename either XML or Lua output will be written.

.. function:: exportScene(opt)

   Convert the current scene (or the selected objects) into an XML tree of SOFA tags.
   The options are conveyed using `opt` object.

   The certain information about the scene is exported, as well as a predefined set of
   objects that facilitate collision detection and solvers in SOFA.

   Camera and lights are exported as well.

   Names of haptic devices are stored in plug-in preferences and not in the scene. Since
   the scene can be shared by many people with varying number of available haptic devices.
   However, the workspace box for the haptic and the surgical instruments are defined in
   the scene


.. function:: exportCamera(o, opt)

   Generate InteractiveCamera tag equivalent of the scene camera (o). Returns
   an ElementTree.Element

.. function:: fovOfCamera(c)

   Auxiliary function to calculate field-of-view of a camera. Blender uses focal length of
   instead of FOV by default. The FOV should also be adjusted to vertical since SOFA uses
   interprets FOV as the vertical field-of-view.

.. function:: objectNode(opt, t)

   If isolating objects is enabled in opt, write t in a separate file, otherwise return t as is

.. function:: exportHaptic(l, opt)

   Export the entire haptic parameters and objects for SOFA. Includes all the specified
   haptic devices in plug-in preferences and the instruments included in the scene
   (should be in the list l).

   Also exports the objects required to enable switching instruments.

.. function:: addConnectionsToTissue(t, o, opt)

   Create the ConnectingTissue tags that connect o to the its o.object1 and o.object2
   if specified. The tags are added to t.
   In current version t is the SolverNode.

.. function:: addConnectionsBetween(t, o, q, opt)

   Create connecting tissue tag between o and q and dump the tags in t

.. function:: exportObject(opt, o)

   export a single Blender object. It calls the appropriate export function based
   on the template of the object. If the object cannot be exported it returns None.

.. function:: exportVisual(o, opt, name = None, with_transform = True)

   export the visual representation of the object. Converts the object to a mesh
   first and then tries to export as much visual data as possible. Including 2D
   texture and color materials.

.. function:: addMaterial(o, t)

   Assuming t is an OglModel tag, add the material parameters from o to t.

.. function:: fixName(name)

   Remove dots from names. Blender usually puts dots in names like Cube.001
   and Cube.002. The dots confuse the SOFA naming system since dots are used
   to reference properties in SOFA.

.. function:: exportTriangularTopology(o, opt)

   Extract the surface mesh from o and create a triangular topology out if it.
   This is usually used for collision models of objects or cloth model.

.. function:: exportRigid(o, opt)

   **DOES NOT WORK AS EXPECTED**

   Export the current model as a rigid object that has collision interaction
   with other objects but does not deform.

   WIP: The exported object is missing some tags to give it complete interaction.
   This can be fixed by looking at some rigid objects in SOFA examples.

.. function:: exportObstacle(o, opt)

   An object that consists only of collision models. It is usually used as
   an anchor point to connect other objects to. It uses a triangular
   topology so it is compatible with almost any object with some sort of mesh
   representation.

.. function:: addElasticityParameters(o, t)

   Auxiliary function used in adding elasticity parameters to most force field tags

.. function:: addTriangularTopology(o, t, opt)

   Similar to exportTriangularTopology, except that instead of creating a new tag
   it adds the topology to an existing tag `t`.

.. function:: exportTriangularTopologyContainer(o, opt)

   Similar to exportTriangularToplogoy, but it creates a different tag that is
   used in cloth

.. function:: exportAttachConstraint(o, opt)

   Export attachments between two objects `o.object1` and `o.object2`. `o` itself is a
   pseudo-object consisting of a sphere that defines the area where the two objects
   need to be linked. The attachments are currently based on springs (specifically
   `StiffSpringForceField` tag).

.. function:: exportCloth(o, opt)

   Export the surface of `o` as a cloth model with provided elasticity parameters.
   It uses triangulation of the surface as a basis.


.. function:: exportInstrument(o, opt)

   Export a rigid object that is controlled using the haptic instrument. The object
   has to be an empty object with multiple parts. Other tags are added to
   enable interactions of the instrument with objects such as carve, suture, etc.

.. function:: exportHexVolumetric(o, opt)

   Export the object `o` as a deformable volumetric objects consisting only of hexahedral
   elements. The object `o` must have hexahedral elements in it for this to work.

   The result is deformable and carvable if enabled.

.. function:: exportVolumetric(o, opt)

   Export the object `o` as a deformable volumetric object consisting only of tetrahedral
   elements.

.. function:: exportThickQuadShell(o, opt)

   Convert the quadrilateral surface `o` (no triangles allowed) to a thick hexahedral
   mesh by offsetting the surface in the normal direction. The inner and outer lining have
   different collision model groups to allow for some self-collision testing without enabling
   the very time-consuming self-collision feature of TriangleModel tag.

.. function:: exportThickCurve(o, opt)

   Convert given curve into a string of hexahedral elements. The inner workings are
   very similar to exportHexVolumetric.

.. function:: addConstraintCorrection(o, t)

   Almost every node needs a constraint correction tag under LCP. This auxiliary
   function adds the appropriate constraint correction tag to an object.

   Currently it chooses  between *UncoupledConstraintCorrection* (default) and more
   expensive *PrecomputeConstraintCorrection* if enabled.

.. function:: exportThickCurveTopology(o, t)

   Generate hexahedra for a thick curve

.. function:: exportThickShellTopologyies(o, opt, name)

   Export a HexahedronSetTopology container with the topology of a
   thick shell. The object is supposed to be convertible to a quad mesh.
   The object can have two custom attributes:
   - thickness: total thickness of the shell multiplied by normal
   - layerCount: total number of layers generated. 3 means 4 layers of surfaces and 3 layers of hexahedral elements.

   Return value is three nodes,
   - Volumetric topology for physical model
   - Outer shell topology
   - Inner shell topology

.. function:: exportHexahedralTopology(o, opt, name)

   Create a hexahedral container for the object `o`. Object `o` must be
   a hexahedral mesh.

.. function:: exportTetrahedralTopology(o, opt, name)

   Create a tetrahedral container for the object `o`.

.. function:: addSolvers(t)

   Add the default solver tags to `t`.

.. function:: createMechanicalObject(o)

   Create a generic mechanical object for Blender object `o`. The
   MechanicalObject tag is always the place to put object transform.
   So this simple function takes care of embedding the affine transformation information
   from Blender to SOFA.

.. function:: rotation_to_quaternion(o)

   Get the rotation of the object as a quaternion in the form of

   .. math:: q = w + x \mathbf{i} + y \mathbf{j} + z \mathbf{k}

   The return value is [x, y, z, w].

.. function:: rotation_to_XYZ_euler(o)

   Get the rotation of the object in XYZ euler format.

.. function:: geometryNode(opt, t)

   Special handling for geometry nodes when needed
   Most of the time this is identity function. But when isolate_geometry
   is enabled it will put the geometry node into a separate file
   and return the node.

.. function:: exportSeparateFile(opt, t, name)

   Export the tag t into a separate file and return the include tag
   for it

.. function:: stringify_etree(node)

   Convert all the attributes of all the elements in the `node` tree
   to strings. This is only needed for XML output, since the XML serializer
   chokes on non-string attributes. For Lua output, the lua_format can handle
   a variety of data types and convert them to Lua literals correctly.

.. function:: ndarray_to_flat_string(a)

   Serialize elements of a NumPy ndarray (multi-dimensional array) to a string without
   any hierarchy. The return strings contains all the element of the array in default
   NumPy order. Consecutive numbers are only separated by space.

.. function:: vector_to_string(v)

   Convert a vector to string

.. function:: iterable(o)

   Auxiliary function to test if the object is iterable

.. class:: ExportException(message)

   The exception that is thrown by the export function if there is an error. The
   message is shown to the user

.. data:: FILEFORMATS

   The items for the enumerations used in selecting between XML and SaLua file formats.


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
The self-contained plug-in for import/export of GMSH files.
Included here for convenience; it can be used by itself as a plugin.


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
