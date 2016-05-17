bl_info = {
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Di Xie",
    'version': (0, 2,  0),
    'blender': (2, 77, 0),
    'location': "https://bitbucket.org/surflab/blender2sofa",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'wiki_url': "https://bitbucket.org/surflab/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/issues",
    'category': 'Import-Export'
}

import bpy

from . import io_msh, ui, conn_tiss, fattytissue, export, runsofa, thick_curve, preferences, types
from .types import *



def menu_func_export(self, context):
    self.layout.operator(export.ExportToSofa.bl_idname, text="SOFA Scene (.scn;.salua)")

addon_keymaps = []


def register():
    # Register the entire module
    bpy.utils.register_module(__name__)
    # Register properties in io_msh
    io_msh.register_other()
    # Add the items in export menu
    bpy.types.INFO_MT_file_export.append(menu_func_export)
    # Add SOFA properties to scene and objects
    types.register_sofa_properties()
    # Add keyboard shortcut F5 for invoking RunSofa
    km = bpy.context.window_manager.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')
    kmi = km.keymap_items.new(runsofa.RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))

def unregister():
    # Remove keyboard shortcuts
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    # Remove SOFA properties
    types.unregister_sofa_properties()
    # Remove items in export menu
    bpy.types.INFO_MT_file_export.remove(menu_func_export)
    # Remove io_msh properties
    io_msh.unregister_other()
    # Unregister everything
    bpy.utils.unregister_module(__name__)

