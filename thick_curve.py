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
