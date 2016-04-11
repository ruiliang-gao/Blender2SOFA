bl_info = {
    'name': "SOFA Export plugin",
    'author': "Saleh Dindar, Di Xie",
    'version': (0, 2,  0),
    'blender': (2, 69, 0),
    'location': "",
    'warning': "",
    'description': "Export Blender scenes into SOFA scene files",
    'wiki_url': "https://bitbucket.org/surflab/blender2sofa/wiki/",
    'tracker_url': "https://bitbucket.org/surflab/blender2sofa/",
    'category': 'Import-Export'
}

import bpy

from . import io_msh, ui, conn_tiss, fattytissue, export, runsofa, thick_curve

class HapticDeviceList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        layout.label(text=item.deviceName, icon='SCULPTMODE_HLT')

class AddHapticDevice(bpy.types.Operator):
    bl_idname = 'other.add_haptic_device'
    bl_label = 'Add Haptic Device'
    def execute(self, context):
        context.user_preferences.addons[__name__].preferences.hapticDevices.add()
        return { 'FINISHED' }

class RemoveHapticDevice(bpy.types.Operator):
    bl_idname = 'other.remove_haptic_device'
    bl_label = 'Remove Haptic Device'
    def execute(self, context):
        p = context.user_preferences.addons[__name__].preferences
        if p.activeHapticDevice >= 0 and p.activeHapticDevice < len(p.hapticDevices):
            p.hapticDevices.remove(p.activeHapticDevice)
        return { 'FINISHED' }

class Blender2SOFASettings(bpy.types.AddonPreferences):
    bl_idname = __name__

    hapticDevices = bpy.props.CollectionProperty(type=ui.HapticProperties)
    hapticEnabled = bpy.props.BoolProperty(name='Haptic Enabled',default=False,description='Enable haptic devices in the generated scenes')
    activeHapticDevice = bpy.props.IntProperty()

    def draw(self, context):
        layout = self.layout
        layout.label(text='Haptic Devices')
        row = layout.row()
        row.template_list('HapticDeviceList', '', self, 'hapticDevices', self, 'activeHapticDevice')
        c = row.column(align=True)
        c.operator('other.add_haptic_device', icon='ZOOMIN', text='')
        c.operator('other.remove_haptic_device', icon='ZOOMOUT', text='')
        if self.activeHapticDevice >= 0 and self.activeHapticDevice < len(self.hapticDevices):
            layout.label('Haptic properties')
            b = layout.box()
            h = self.hapticDevices[self.activeHapticDevice]
            b.prop(h, 'deviceName')
            b.prop(h, 'scale')
            b.prop(h, 'forceFeedback')
            if h.forceFeedback:
                b.prop(h, 'forceScale')


def menu_func_export(self, context):
    self.layout.operator(runsofa.ExportToSofa.bl_idname, text="SOFA Scene (.scn;.salua)")

addon_keymaps = []

def register():
    bpy.utils.register_module(__name__)
    io_msh.register()
    bpy.types.INFO_MT_file_export.append(menu_func_export)
    bpy.types.Scene.sofa = bpy.props.PointerProperty(type=ui.SOFASceneProperties)
    bpy.types.Object.sofaprops = bpy.props.PointerProperty(type=ui.SOFAObjectProperties)

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
    del bpy.types.Object.sofaprops
    io_msh.unregister()
    bpy.types.INFO_MT_file_export.remove(menu_func_export)
    bpy.utils.unregister_module(__name__)

