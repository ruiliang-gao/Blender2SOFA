===============================
SOFA Annotations and Parameters
===============================


Annotations
===========

The following are the differnt types of annotations that can be picked from the drop down box at in the *SOFA Annotations* panel.

----------
Volumetric
----------

An interactive object where the volume inside of it is filled with hexahedra or tetrahedra.
If the object annotated as volumetric does not have any hexahedra or tetrahedra, it will show a
warning in the annotations panel and give you an error if you try to export it to SOFA.

A volumetric object has elasticity_ and collision_ properties, in addition to attachment_ specifications.

-----------
Thick Shell
-----------

An interactive object where the surface is layered with hexahedra. In Blender, the object tagged will
not have any hexahedra. The surface must consist only of quads so it can be converted to a hexahedral mesh
when exported. To control the generation of the volumetric mesh, two parameters can be set:

* *layerCount*: Number of layers of hexahedra that are created from each quad.
* *thickness* : The thickness of the shell, determines the thickness of hexahedra along the normal direction of the surface

Thick shell supports elasticity_, collision_ and attachment_ properties like volumetric.

-----------
Thick Curve
-----------

An interactive hexahedral object created from a Bezier curve. In Blender, the object will not have any hexahedra;
instead it is a Bezier curve with depth.

Thick curve supports elasticity_, collision_ and attachment_ properties like volumetric.

-----
Cloth
-----

An elastic surface object. The appearance and interactivity is exactly as the surface mesh in Blender.
Elasticiy is defined using elasticity_ parameters.

Cloth also supports collision_ and attachment_ properties.

--------
Obstacle
--------

A non-interactive solid object that prevents other objects going through. It is usually used for
supporting other objects. Other objects can be also attached to it using `Spring Attachment`_ or attachment_ property on the other object.

Obstacle only supports collision_ properties.

----------------------------------
Sphere Constraint / Box Constraint
----------------------------------

A guide object that defines the area where the nodes of its parent object should be fixed. The object has to be an empty object displayed as a sphere/box. A constraint object must be placed inside another interactive object (Volumetric, Thick Shell, Thick Curve, or Cloth). The constraint object will not be visible in the SOFA simulation, it only changes the behavior of its parent.

-----------------
Spring Attachment
-----------------

A guide object that defines spring attachment between two objects. It supports attachment_ properties
but the two objects specified will be connected to one another in the area that overlaps the spring attachment sphere.

The spring attachment does not have any behavioral or visual appearance in the SOFA simulation.

-----------------
Haptic Instrument
-----------------

A rigid object that is controlled by haptic force-feedback devices. Haptic instruments are only included in the SOFA simluation if haptic devices are set-up.

The haptic instrument can be used to probe/nudge other objects in the scene. The instrument can be used to manipulate the scene by clicking or holding a button on the haptic input device. Dependening on the value of *Function*, the effects are different:

* *Grasp*: grasp objects and pull
* *Suture*: suture two parts of one objects with a thread
* *Carve*: destroy tissue
* *Clamp*: Clip applier

Haptic instrument always has parts, for that reason, the object tagged as haptic instrument must be an empty object
with some mesh objects as its children. Each part may be tagged is *Instrument Part*.

---------------
Instrument Part
---------------

Defines an animated part of the instrument. Left jaw rotates to left, right jaw rotates to right; for other parts the fixed is used. If a part of instrument is not annotated as Instrument Part, it will be considered fixed.

--------------
Instrument Tip
--------------

The only collision part of the instrument. Any part of the instrument that is not tagged as Instrument Tip
is purely visual. Mesh points of the instrument tip will be used for collision detection and are the primary point
of interaction with the other objects. The instrument tip object is a behavioral object and does not have any visual
appearance

------
Visual
------

The default annotation for all other objects, this means that the object would appear only as
a visual element and will not be interactive. If you don't want the object to appear in the
SOFA scene, then click on the camera next to the object name in the outline view to exclude the
object from rendering.


Parameters
==========

--------------------
Collision Parameters
--------------------

.. _collision:

* *collisionGroup*
* *selfCollision*
* *contactFriction*
* *contactStiffness*
* *Interactive*: Allow the object to be manipulated by the haptic instruments
* *Carvable*: Allow destroying parts of the object using mouse or haptic instruments


---------------------
Elasticity Parameters
---------------------

.. _elasticity:

* *Stiffness (Young Modulus)*: Stretch stiffness (a.k.a `Young Modulus`_). Lower values result in soft objects. Higher values make the object more rigid.
* *Compressability*: a.k.a `Poisson Ratio`_. The default value works for most objects.
* *Rayleigh Stiffness*: Affects the damping of the object. Higher values make the object move slowly. Lower values make it snappy, but sometimes unstable
* *Bending Stiffness*: Only applies to Cloth_, the resistance in bending the surface. A value of 0 makes the cloth behave very fluid like smooth silk. Higher values make it more stiff like rubber.
* *Damping*: Another damping parameter, only works on cloth
* *Accurate Constraints*: More accurate constraints but requires lengthy precomputation the first time the simulation is started.


---------------------
Attachment Parameters
---------------------

.. _attachment:

For `Spring Attachment`_, the first object and second object are attached to one another with springs.
For other types of objects, the main object (for which the properties are set) is attached to first object
and second object independently (either one or both may be left blank).

* *Attach Stiffness*: Hookes constant of the springs that are created between the objects
* *First Object*: Name of the first object to be attached
* *Second Object*: Name of the second object to be attached


--------------------
Other Parameters
--------------------
* *3D Texture*: filename of the 3D texture for the volumetric object. The 3D texture coordinates are generated automatically
