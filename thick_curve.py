#!/usr/bin/env python
# -*- coding: utf-8 -*-

import bpy

class AddThickCurve(bpy.types.Operator):
    bl_idname = "mesh.add_thick_curve"
    bl_label = "Add Thick Curve"
    bl_options = { 'UNDO' }
    bl_description = 'Add a curve with a bevel object to give it volume'
    
    
    radius1 : bpy.props.FloatProperty(name = 'Radius1', description = 'Thickness of one end of the curve.',default=1.0,min=0,max=10)
    radius2 : bpy.props.FloatProperty(name = 'Radius2', description = 'Thickness of the other end of the curve.',default=1.0,min=0,max=10)
    taper_curve : bpy.props.BoolProperty(name = 'Taper curve',default=True,description='If set, you may adjust ratio of max and min radii.')
    

    @classmethod
    def poll(self, context):
        return context.scene is not None
    
    def check(self, context):
        return True
        
    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)
        
    def draw(self, context):
        l = self.layout
        l.prop(self, 'taper_curve')
        if self.taper_curve:
          l.prop(self, 'radius1')
          l.prop(self, 'radius2')

    def execute(self, context):
        
        if self.taper_curve:
          R1 = self.radius1
          R2 = self.radius2
          #create and edit model curve for tapering
          bpy.ops.curve.primitive_bezier_curve_add()
          bpy.context.object.hide_render = True
          for obj in bpy.context.selected_objects:
            obj.name = 'ModelCurve' #rename curve
            modelcurve = obj.name
          bpy.ops.object.editmode_toggle()
          bpy.ops.curve.select_all(action='TOGGLE')
          bpy.context.active_object.data.splines[0].bezier_points[0].select_control_point = True
          bpy.ops.transform.translate(value = (0, R1, 0))
          bpy.context.active_object.data.splines[0].bezier_points[0].select_control_point = False
          bpy.context.active_object.data.splines[0].bezier_points[1].select_control_point = True
          bpy.ops.transform.translate(value = (0, R2, 0))
          bpy.ops.object.editmode_toggle()
        
        # Add a Bezier object
        bpy.ops.curve.primitive_bezier_curve_add()
        for obj in bpy.context.selected_objects:
            obj.name = 'ThickCurve' #rename curve
            thickcurve = obj.name
        c1 = context.object.data
        c1.bevel_depth = 0.1
        c1.fill_mode = 'FULL'
        context.object.template = 'THICKCURVE'
        context.object.carvable = True
        
        if self.taper_curve:
          # Taper the new object
          bpy.context.object.data.taper_object = bpy.data.objects[modelcurve]
          
        bpy.data.objects[modelcurve].select_set(True)
        bpy.data.objects[thickcurve].select_set(True)
        bpy.context.view_layer.objects.active = bpy.data.objects[thickcurve]
        bpy.ops.object.parent_set()

        return { 'FINISHED' }
        
# class HapticOptions(bpy.types.Operator):
    # bl_idname = "option.show_haptic_options"
    # bl_label = "Show Haptic options"
    # bl_options = { 'UNDO' }
    # bl_description = 'Show Haptic options for configuration'

    # @classmethod
    # def poll(self, context):
        # return context.scene is not None

    # def execute(self, context):
        # bpy.context.preferences.active_section = 'ADDONS'
        # bpy.ops.screen.userpref_show('INVOKE_DEFAULT')
        # bpy.data.window_managers["WinMan"].addon_filter = 'User'

        # return { 'FINISHED' }
