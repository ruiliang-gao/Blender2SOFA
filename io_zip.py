# This file can be used as a stand-alone plugin
# but here it is used in the Blened2SOFA package
bl_info = {
    'name': "Import/Export SOFA scene zip files plugin",
    'author': "Jose Carlos de Almeida Machado",
    'version': (0, 1, 0),
    'blender': (2, 80, 0),
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
import shutil

class ImportZIP(bpy.types.Operator, ImportHelper):
  """Import blend files from zip files"""
  bl_idname = "import_scene.zip"
  bl_label = "Import SCN"
  bl_options = {'UNDO'}


  filename_ext = ".zip"
  filter_glob: StringProperty(default="*.zip", options={'HIDDEN'})
  
  # This property defines if uses the current scene or erase it
  
  erase_scene: BoolProperty(
            name="Erase Existing scene",
            description="Erases existing scene",
            default=False,
            )
   
  def execute(self, context):
    #here
    filepath = self.filepath
    fileDirectory = os.path.dirname(filepath)
    tempPath = fileDirectory +"\\temp_zip_extracted_blend_files"
    scn = bpy.context.scene
    archive = zipfile.ZipFile(filepath, 'r')
	
	#check if the the option to erase the scene is checked
    if(self.erase_scene):
      for ob in bpy.context.scene.objects:
        ob.select_set(True)
      bpy.ops.object.delete()
	
	
	#file names inside the zip file
    for file in archive.namelist():
	  #extracts files ending with .blend
      if file.endswith(".blend"):
        extractedFile = archive.extract(file,tempPath)
		#load objects to blender
        with bpy.data.libraries.load(extractedFile) as (data_from, data_to):
          data_to.objects = data_from.objects
	    #link object to current scene
        for obj in data_to.objects:
          if obj is not None:
            scn.collection.objects.link(obj)
			#select the new objects in the scene
            obj.select_set(True)
			
	#if the scene was erased, deselect all.		
    if(self.erase_scene):
      for obj in scn:
        obj.select_set(False)
		
    #deletes the temporary created folder
    shutil.rmtree(tempPath)	
	
    return { 'FINISHED' }



def menu_func_import(self, context):
    self.layout.operator(ImportZIP.bl_idname, text="SOFA Scene (.zip)")

def register_other():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)
    # bpy.types.INFO_MT_file_import.append(menu_func_import)
   

def unregister_other():
    bpy.types.TOPBAR_MT_file_import.remove(menu_func_import)


def register():
    register_other()
    bpy.utils.register_module(__name__)

def unregister():
    unregister_other()
    bpy.utils.unregister_module(__name__)
