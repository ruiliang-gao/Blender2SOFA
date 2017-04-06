import bpy

from .types import *


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
        layout.operator("option.show_haptic_options", icon= 'SETTINGS')
        #layout.separator()
        layout.label('Create')
        c = layout.column(align=True)
        #c.operator("mesh.construct_connecting_tissue", icon='OUTLINER_OB_META', text='Connecting Tissue')
        layout.separator()
        c.operator("mesh.construct_fatty_tissue", icon='FACESEL_HLT', text = 'Fatty Tissue')
        # layout.separator()
        layout.operator("mesh.add_thick_curve", icon= 'ROOTCURVE')
        #layout.operator("mesh.add_hex_rod", icon= 'ROOTCURVE')
        

        
        if ConvertFromCustomProperties.poll(context):
            layout.operator(ConvertFromCustomProperties.bl_idname)
    

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
        p = o
        layout = self.layout
        layout.prop(p, 'template', text='')

        t = p.template
        c = layout.column(align=True)
        c.prop(p, 'texture2d')
        if t == 'INSTRUMENT':
            c.prop(p, 'toolFunction')
        elif t == 'INSTRUMENTPART':
            c.prop(p, 'instrumentPart')
        elif t in ['INSTRUMENTTIP', 'INSTRUMENTCOLLISION']:
            c.prop(p, 'proximity')
        elif t in [ 'VOLUMETRIC', 'THICKSHELL', 'THICKCURVE' ]:
            c.prop(p, 'youngModulus')
            c.prop(p, 'poissonRatio')
            c.prop(p, 'damping')
            c.prop(p, 'rayleighStiffness')
            if t == 'THICKSHELL':
                c.prop(p, 'thickness')
                c.prop(p, 'layerCount')
            if t == 'THICKCURVE' and p.type != 'MESH':
                c.prop(o.data, 'bevel_depth', text = 'Thickness')
            c.prop(p, 'texture3d')
            c.prop(p, 'precomputeConstraints')

            c = layout.column(align=True)
            c.label('Attached Objects')
            c.prop_search(p, "object1", context.scene, "objects")
            c.prop_search(p, "object2", context.scene, "objects")
            c.prop(p, 'attachThreshold')
            c.prop(p, 'attachStiffness')
            c.prop(p, 'naturalLength')
        elif t == 'CLOTH':
            c.prop(p, 'youngModulus')
            c.prop(p, 'bendingStiffness')
            c.prop(p, 'damping')
            c.prop(p, 'precomputeConstraints')
        elif t == 'ATTACHCONSTRAINT'  :
            c.prop(p, 'attachStiffness')
            c.prop(p, 'attachThreshold')
            c.prop(p, 'naturalLength')
            c.prop_search(p, 'object1', context.scene, "objects")
            c.prop(p, 'alwaysMatchForObject1')
            c.prop_search(p, 'object2', context.scene, "objects")
            c.prop(p, 'alwaysMatchForObject2')

        if t in [ 'VOLUMETRIC', 'CLOTH', 'THICKSHELL', 'THICKCURVE' ]:
            c = layout.column(align=True)
            c.label('Collision Parameters')
            c.prop(p, 'collisionGroup')
            c.prop(p, 'contactFriction')
            c.prop(p, 'contactStiffness')
            c.prop(p, 'selfCollision')
            c.prop(p, 'carvable')
            c.prop(p, 'suture')

        if t in [ 'VOLUMETRIC', 'THICKSHELL', 'THICKCURVE', 'COLLISION', 'SAFETYSURFACE', 'VISUAL' ]:
            c.prop(p, 'useShader')
            c.prop(p, 'shaderFile')
            c.prop(p, 'useTessellation')
        
        

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
        c.prop(s, "mu")
        c.prop(s, "alarmDistance")
        c.prop(s, "contactDistance")
        c.prop(s, "showXYZFrame")
        c.prop(s, "precompution")
        c.prop(s, "useSpeechRecognition")
        c.prop_search(s, "hapticWorkspaceBox", context.scene, "objects")
        c.prop_search(s, "defaultInstrument", context.scene, "objects")
        c.prop(s, "veinForceThreshold")


PROPERTY_NAME_MAP = { 'topObject': 'object1', 'botObject': 'object2', 'stretchDamping' : 'damping',
    'attach_stiffness': 'attachStiffness', '3dtexture':'texture3d' }
TEMPLATE_MAP = { 'CONNECTIVETISSUE': 'VOLUMETRIC',
    # the rest are identity mappings
    'THICKSHELL': 'THICKSHELL', 'CLOTH': 'CLOTH','COLLISION':'COLLISION','ATTACHCONSTRAINT':'ATTACHCONSTRAINT','SPHERECONSTRAINT':'SPHERECONSTRAINT',
    'VOLUMETRIC':'VOLUMETRIC','INSTRUMENT':'INSTRUMENT','INSTRUMENTPART':'INSTRUMENTPART','INSTRUMENTTIP':'INSTRUMENTTIP','THICKCURVE':'THICKCURVE',
    'INSTRUMENTCOLLISION':'INSTRUMENTCOLLISION', 'SAFETYSURFACE':'SAFETYSURFACE'}

def removeCustomProperty(o, k):
    del o[k]
    rna = o.get('_RNA_UI',{})
    if k in rna:
        del rna[k]


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
        
class ConvertFromCustomProperties(bpy.types.Operator):
    bl_idname = "scene.convert_from_custom_properties"
    bl_label = "Recover from Older version of Blender2SOFA"
    bl_description = "Automatically recover annotations and properties from older version of Blender2SOFA that used custom properties"


    @classmethod
    def poll(self, context):
        return context.scene is not None and context.scene.get('sofa') != None or context.object is not None and context.object.get('annotated_type') != None

    def execute(self, context):
        scene = context.scene
        # Convert scene properties
        removeCustomProperty(scene, 'sofa')
        for o in scene.objects:
            template = o.get('annotated_type')
            # set the object template
            if template in TEMPLATE_MAP:
                removeCustomProperty(o, 'annotated_type')
                o.template = TEMPLATE_MAP[template]
                for k,v in o.items():
                    if k in PROPERTY_NAME_MAP:
                        setattr(o, PROPERTY_NAME_MAP[k], v)
                        removeCustomProperty(o, k)
        return { 'FINISHED' }
