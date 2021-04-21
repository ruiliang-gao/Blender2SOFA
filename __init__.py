bl_info = {
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Ruiliang Gao, Di Xie",
    'version': (0, 2,  1),
    'blender': (2, 90, 0),
    'location': "https://bitbucket.org/surflab/blender2sofa",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'doc_url': "https://bitbucket.org/surflab/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/issues",
    'category': 'Import-Export'
}

import bpy

from . import io_msh, io_zip, ui, conn_tiss, hex_rod, fattytissue, export, runsofa, thick_curve, preferences, types, conn_tiss
from . types import *

addon_keymaps = []

classes = ( 
    types.HapticProperties, 
    preferences.Blender2SOFASettings,
    export.ExportToSofa, 
    conn_tiss.OBJECT_PT_ConnectingTissuePanel, conn_tiss.OBJECT_OT_ShrinkwrapTest, conn_tiss.OBJECT_OT_ConnectingTissue, 
    ui.SOFA_PT_Actions, ui.SOFA_PT_AnnotationPanel, ui.SOFA_PT_PropertyPanel, ui.GenerateFixedConstraints, ui.ExportObjToSofa, ui.ConvertFromCustomProperties, 
    fattytissue.FattyTissue, 
    # hex_rod.HexRod, 
    io_msh.MeshTetrahedron, io_msh.MeshHexahedron, io_msh.VolumetricMeshPanel, io_msh.ReCalculateOuterSurface, io_msh.RemoveDegenerateHexahedra, io_msh.ExportMSHOperator, io_msh.IO_OT_import_mesh, 
    io_zip.ImportZIP, 
    preferences.HAPTIC_UL_DeviceList, preferences.AddHapticDevice, preferences.RemoveHapticDevice, 
    runsofa.RunSofaOperator, 
    thick_curve.AddThickCurve, 
    ui.HapticOptions )


def register():
    for c in classes:
        bpy.utils.register_class(c)
    # Add SOFA properties to scene and objects
    types.register_sofa_properties()
    # Register UI
    ui.register_other()
    # Register properties in io_msh
    io_msh.register_other()
    # Register properties in io_zip
    io_zip.register_other()
    # Register properties in export
    export.register_other()
    
    # Add keyboard shortcut F5 for invoking RunSofa
    km = bpy.context.window_manager.keyconfigs.addon.keymaps.new(name='Object Mode', space_type='EMPTY')
    kmi = km.keymap_items.new(runsofa.RunSofaOperator.bl_idname, 'F5', 'PRESS')
    addon_keymaps.append((km, kmi))

def unregister():
    for c in classes:
        bpy.utils.unregister_class(c)
    # Remove keyboard shortcuts
    for km, kmi in addon_keymaps:
        km.keymap_items.remove(kmi)
    addon_keymaps.clear()
    # Remove SOFA properties
    types.unregister_sofa_properties()
    # Remove items in export menu
    export.unregister_other()
    # Remove io_zip properties
    io_zip.unregister_other()
    # Remove io_msh properties
    io_msh.unregister_other()

if __name__ == "__main__":
    register()