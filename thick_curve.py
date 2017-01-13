#!/usr/bin/env python
# -*- coding: utf-8 -*-

import bpy

class AddThickCurve(bpy.types.Operator):
    bl_idname = "mesh.add_thick_curve"
    bl_label = "Add Thick Curve"
    bl_options = { 'UNDO' }
    bl_description = 'Add a curve with a bevel object to give it volume'

    @classmethod
    def poll(self, context):
        return context.scene is not None

    def execute(self, context):
        # Add a Bezier object
        bpy.ops.curve.primitive_bezier_curve_add()
        c = context.object.data
        c.bevel_depth = 0.1
        c.fill_mode = 'FULL'
        context.object.template = 'THICKCURVE'
        context.object.carvable = True

        return { 'FINISHED' }
        
class HapticOptions(bpy.types.Operator):
    bl_idname = "option.show_haptic_options"
    bl_label = "Show Haptic options"
    bl_options = { 'UNDO' }
    bl_description = 'Show Haptic options for configuration'

    @classmethod
    def poll(self, context):
        return context.scene is not None

    def execute(self, context):
        bpy.context.user_preferences.active_section = 'ADDONS'
        bpy.ops.screen.userpref_show('INVOKE_DEFAULT')
        bpy.data.window_managers["WinMan"].addon_filter = 'User'

        return { 'FINISHED' }
