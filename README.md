# Blender to SOFA export plugin

![blender2sofa-logo.png](https://bitbucket.org/repo/Ayy6LE/images/4037932839-blender2sofa-logo.png)

Utilize Blender for modeling organs, tools. Set-up the lighting. Create 
SOFA scenes from a Blender scene easily. UI is provided for specifying
parameters that are needed in a SOFA simulation.

## Install
1. Download latest distribution zip file [surflab-blender2sofa-latest.zip](https://bitbucket.org/surflab/blender2sofa/get/default.zip)
2. Use _Install From File..._ in _Addons_ tab of Blender to install the plugin
3. Activate the plugin from the list. Hit _Save User Settings_ to make it permanent.

[Installation intstructions with pictures](https://bitbucket.org/surflab/blender2sofa/wiki/Install)

### Tetrahedral Meshes
Blender2SOFA does not create tetrahedral meshes any more. You have to create
tetrahedral models using [QTetraMesher](http://qtm.dennis2society.de/) and import
them into your scene using _Import MSH_ operator provided.

## Usage
To export to a Sofa readable .SCN file, go to _File -> Export -> Export To Sofa_, and then in the save dialog
specify the name of your .SCN file

To launch Sofa simulation directly from Blender, hit _F5_

## Supported functionality
Tutorials and documentation are avaliable at http://www.cise.ufl.edu/~dxie/mastersproject/index.shtml