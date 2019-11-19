#!/usr/bin/env python
# -*- coding: utf-8 -*-
import bpy
from .types import HapticProperties

class HAPTIC_UL_DeviceList(bpy.types.UIList):
    def draw_item(self, context, layout, data, item, icon, active_data, active_propname, index):
        layout.label(text=item.deviceName, icon='SCULPTMODE_HLT')

class AddHapticDevice(bpy.types.Operator):
    bl_idname = 'other.add_haptic_device'
    bl_label = 'Add Haptic Device'
    def execute(self, context):
        context.preferences.addons[__package__].preferences.hapticDevices.add()
        return { 'FINISHED' }

class RemoveHapticDevice(bpy.types.Operator):
    bl_idname = 'other.remove_haptic_device'
    bl_label = 'Remove Haptic Device'
    def execute(self, context):
        p = context.preferences.addons[__package__].preferences
        if p.activeHapticDevice >= 0 and p.activeHapticDevice < len(p.hapticDevices):
            p.hapticDevices.remove(p.activeHapticDevice)
        return { 'FINISHED' }

class Blender2SOFASettings(bpy.types.AddonPreferences):
    bl_idname = __package__

    hapticDevices: bpy.props.CollectionProperty(type=HapticProperties)
    hapticEnabled: bpy.props.BoolProperty(name='Haptic Enabled',default=False,description='Enable haptic devices in the generated scenes')
    activeHapticDevice: bpy.props.IntProperty()

    def __init__(self):
        pass
        #self.hapticDevices = bpy.props.CollectionProperty(type=HapticProperties)
        #self.hapticEnabled = bpy.props.BoolProperty(name='Haptic Enabled',default=False,description='Enable haptic devices in the generated scenes')
        #self.activeHapticDevice = bpy.props.IntProperty()

    def draw(self, context):
        layout = self.layout
        layout.label(text='Haptic Devices')
        row = layout.row()
        row.template_list('HAPTIC_UL_DeviceList', '', self, 'hapticDevices', self, 'activeHapticDevice')
        c = row.column(align=True)
        c.operator('other.add_haptic_device', icon='ADD', text='')
        c.operator('other.remove_haptic_device', icon='REMOVE', text='')
        if self.activeHapticDevice >= 0 and self.activeHapticDevice < len(self.hapticDevices):
            layout.label(text='Haptic properties')
            b = layout.box()
            h = self.hapticDevices[self.activeHapticDevice]
            b.prop(h, 'deviceName')
            b.prop(h, 'scale')
            b.prop(h, 'forceFeedback')
            if h.forceFeedback:
                b.prop(h, 'forceScale')
