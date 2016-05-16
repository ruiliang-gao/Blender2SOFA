.. Blender2SOFA documentation master file, created by
   sphinx-quickstart on Mon May 16 12:56:12 2016.
   You can adapt this file completely to your liking, but it should at least
   contain the root `toctree` directive.

Blender to SOFA export plugin
========================================

.. image:: https://bitbucket.org/repo/Ayy6LE/images/4037932839-blender2sofa-logo.png
   :alt: blender2sofa logo

Utilize Blender for modeling organs, tools. Set-up the lighting. Create 
SOFA scenes from a Blender scene easily. UI is provided for specifying
parameters that are needed in a SOFA simulation.

Install
-------
1. Download latest distribution zip file `surflab-blender2sofa-latest.zip`_
2. Use *Install From File...* in *Addons* tab of Blender to install the plugin
3. Activate the plugin from the list. Hit *Save User Settings* to make it permanent.

For more help, look at `Installation intstructions with pictures`_

.. _Installation intstructions with pictures: https://bitbucket.org/surflab/blender2sofa/wiki/Install

.. _surflab-blender2sofa-latest.zip: https://bitbucket.org/surflab/blender2sofa/get/default.zip

Tetrahedral Meshes
------------------
Blender2SOFA does not create tetrahedral or hexahedral meshes. You have to create
tetrahedral models using QTetraMesher_ and import
them into your scene using *Import MSH* operator provided.

.. _QTetraMesher: http://qtm.dennis2society.de/

Usage
-----
To export to a Sofa readable .SCN file, go to *File -> Export -> Export To Sofa*, and then in the save dialog
specify the name of your .SCN file

To launch Sofa simulation directly from Blender, hit *F5*

To learn about how to annotate Blender object to behave a certain way in SOFA, look at :doc:`Annotations`.

Contents:

.. toctree::
   :maxdepth: 2

  Annotations
  Structure
  
Indices and tables
==================

* :ref:`genindex`
* :ref:`modindex`
* :ref:`search`

