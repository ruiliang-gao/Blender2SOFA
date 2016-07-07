# This file can be used as a stand-alone plugin
# but here it is used in the Blened2SOFA package
bl_info = {
    'name': "Import/Export SOFA scene zip files plugin",
    'author': "Jose Carlos de Almeida Machado",
    'version': (0, 1, 0),
    'blender': (2, 69, 0),
    'location': "https://bitbucket.org/surflab/blender2sofa/src/default/io_zip.py",
    'warning': "",
    'description': "Import/export SOFA scene zip files",
    'wiki_url': "https://bitbucket.org/surflab/blender2sofa/wiki/IO_ZIP",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/issues",
    'category': "Import-Export",
}

import bpy
import bpy_extras
from bpy_extras.io_utils import ImportHelper
from bpy.props import StringProperty, BoolProperty
import os
import zipfile

class ImportZIP(bpy.types.Operator, ImportHelper):
  """Import blend files from zip files"""
  bl_idname = "import_scene.zip"
  bl_label = "Import SCN"
  bl_options = {'UNDO'}


  filename_ext = ".zip"
  filter_glob = StringProperty(default="*.zip", options={'HIDDEN'})
  
  # This property defines if uses the current scene or erase it
  
  erase_scene = BoolProperty(
            name="Erase Existing scene",
            description="Erases existing scene",
            default=False,
            )
   
  def execute(self, context):
    #here
    filepath = self.filepath
    fileDirectory = os.path.dirname(filepath)
    scn = bpy.context.scene
    objName = os.path.basename(filepath)#bpy.path.display_name()
    archive = zipfile.ZipFile(self.filepath, 'r')
    filesToDelete=[]
	
    if(self.erase_scene):
      for ob in bpy.context.scene.objects:
        ob.select = True
      bpy.ops.object.delete()
	
	
	#file names inside the zip file
    for file in archive.namelist():
      
      if file.endswith(".blend"):
        extractedFile = archive.extract(file,fileDirectory)
        filesToDelete.append(extractedFile)
        with bpy.data.libraries.load(extractedFile) as (data_from, data_to):
          data_to.objects = data_from.objects
	#link object to current scene
        for obj in data_to.objects:
          if obj is not None:
            scn.objects.link(obj)
    #here 2
    for file in filesToDelete:
      os.remove(file)
		
    return { 'FINISHED' }



def menu_func_import(self, context):
    self.layout.operator(ImportZIP.bl_idname, text="SOFA Scene (.zip)")

def register_other():
    bpy.types.INFO_MT_file_import.append(menu_func_import)
   

def unregister_other():
    bpy.types.INFO_MT_file_import.remove(menu_func_import)


def register():
    register_other()
    bpy.utils.register_module(__name__)

def unregister():
    unregister_other()
    bpy.utils.unregister_module(__name__)
