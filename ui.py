import bpy

from .types import *

class SofaObjectAnnotationPanel(bpy.types.Panel):
    """A panel to adjust object properties"""
    bl_label = "SOFA annotations"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    #bl_context = 'sofa'

    @classmethod
    def poll(self, context):
        return context.object is not None

    def draw(self, context):
        o = context.object
        p = o.sofaprops
        layout = self.layout
        layout.prop(p, 'template', text='')

        t = p.template
        c = layout.column(align=True)
        if t == 'INSTRUMENT':
            c.prop(p, 'toolFunction')
        elif t == 'INSTRUMENTPART':
            c.prop(p, 'instrumentPart')
        elif t in [ 'VOLUMETRIC', 'THICKSHELL', 'THICKCURVE' ]:
            c.prop(p, 'youngModulus')
            c.prop(p, 'poissonRatio')
            c.prop(p, 'damping')
            c.prop(p, 'rayleighStiffness')
            if t == 'THICKSHELL':
                c.prop(p, 'thickness')
                c.prop(p, 'layerCount')
            if t == 'THICKCURVE':
                c.prop(o.data, 'bevel_depth', text = 'Thickness')
            c.prop(p, 'texture3d')
            c.prop(p, 'precomputeConstraints')

            if p.object1 != '' or p.object2 != '':
                c = layout.column()
                c.prop(p, 'attachStiffness')
                c.prop_search(p, "object1", context.scene, "objects")
                c.prop_search(p, "object2", context.scene, "objects")
        elif t == 'CLOTH':
            c.prop(p, 'youngModulus')
            c.prop(p, 'bendingStiffness')
            c.prop(p, 'damping')
            c.prop(p, 'precomputeConstraints')
        elif t == 'ATTACHCONSTRAINT'  :
            c.prop(p, 'attachStiffness')
            c.prop_search(p, 'object1', context.scene, "objects")
            c.prop(p, 'alwaysMatchForObject1')
            c.prop_search(p, 'object2', context.scene, "objects")
            c.prop(p, 'alwaysMatchForObject2')

        if t in [ 'VOLUMETRIC', 'CLOTH', 'THICKSHELL', 'THICKCURVE' ]:
            c = layout.column(align=True)
            c.prop(p, 'collisionGroup')
            c.prop(p, 'contactFriction')
            c.prop(p, 'contactStiffness')
            c.prop(p, 'selfCollision')
            c.prop(p, 'carvable')
            c.prop(p, 'suture')
          

        if t == 'VOLUMETRIC':
            if o.type != 'MESH' or len(o.data.hexahedra) + len(o.data.tetrahedra) == 0:
                layout.label('This object does not contain a volumetric mesh', icon='ERROR')

        if t == 'THICKCURVE':
            if o.type != 'CURVE':
                layout.label('This object is not a curve', icon='ERROR')


class SofaScenePropertyPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "SOFA Scene Properties"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    @classmethod
    def poll(self, context):
        return context.scene is not None

    def draw(self, context):
        layout = self.layout
        s = context.scene
        c = layout.column(align=True)
        c.prop(s.sofa, "mu")
        c.prop(s.sofa, "alarmDistance")
        c.prop(s.sofa, "contactDistance")
        c.prop(s.sofa, "showXYZFrame")

class SofaActionsPanel(bpy.types.Panel):
    bl_label = "SOFA Actions"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'

    @classmethod
    def poll(self, context):
        return context.scene is not None

    def draw(self, context):
        layout = self.layout

        layout.operator("scene.runsofa", icon='PLAY')
        layout.separator()
        layout.label('Create')
        c = layout.column(align=True)
        c.operator("mesh.construct_connecting_tissue", icon='OUTLINER_OB_META', text='Connecting Tissue')
        c.operator("mesh.construct_fatty_tissue", icon='FACESEL_HLT', text = 'Fatty Tissue')
        layout.separator()
        layout.operator("mesh.add_thick_curve", icon= 'ROOTCURVE')



