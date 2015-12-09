bl_info = { 
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Di Xie",
    'version': (0, 1,  2),
    'blender': (2, 74, 0),
    'location': "",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'wiki_url': "https://bitbucket.org/surflab/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/",
    'category': 'Mesh'
}

import bpy

from .io_msh import *
from .ui import *
from .conn_tiss import *
from .export import *
from .runsofa import *

############## Register/Unregister add-on ###########################################

# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportToSofa.bl_idname, text="SOFA Scene (.scn;.salua)")

addon_keymaps = []

def register():
    io_msh.register()
    ui.register()
    conn_tiss.register()

    bpy.utils.register_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.append(menu_func_export)

    bpy.utils.register_class(RunSofaOperator)

    # handle the keymap
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')

    kmi = km.keymap_items.new(RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))



def unregister():
    bpy.utils.unregister_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)

    # handle the keymap
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
        
    bpy.utils.unregister_class(RunSofaOperator)

    io_msh.unregister()
    ui.unregister() 
    conn_tiss.unregister()
