bl_info = { 
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Di Xie",
    'version': (0, 1,  2),
    'blender': (2, 69, 0),
    'location': "",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'wiki_url': "https://bitbucket.org/surflab/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/",
    'category': 'Import-Export'
}

import bpy

from . import io_msh, ui, conn_tiss, hex_rod, fatty_tissue, export, runsofa


def menu_func_export(self, context):
    self.layout.operator(runsofa.ExportToSofa.bl_idname, text="SOFA Scene (.scn;.salua)")

addon_keymaps = []

def register():
    bpy.utils.register_module(__name__)
    io_msh.register()
    bpy.types.INFO_MT_file_export.append(menu_func_export)
    bpy.types.Scene.sofa = bpy.props.PointerProperty(type=ui.SOFASceneProperties)

    # Add keyboard shortcut F5 for invoking RunSofa
    wm = bpy.context.window_manager
    km = wm.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')
    kmi = km.keymap_items.new(runsofa.RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))

def unregister():
    # Remove keyboard shortcuts
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()

    del bpy.types.Scene.sofa
    io_msh.unregister()
    bpy.types.INFO_MT_file_export.remove(menu_func_export)
    bpy.utils.unregister_module(__name__)

