import bpy



class SofaObjectAnnotationPanel(bpy.types.Panel):
    """A panel to adjust objeRESTRICT_RENDER_OFFct properties"""
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
        c = layout.column_flow(align=True,columns=1)
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
                c = layout.column_flow(columns=1)
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
            c = layout.column_flow(align=True,columns=1)
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
        c = layout.column_flow(align=True, columns=1)
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
        c = layout.column_flow(align=True,columns=1)
        c.operator("mesh.construct_con_tissue", icon='OUTLINER_OB_META', text='Connecting Tissue')
        c.operator("mesh.construct_fatty_tissue", icon='MOD_MESHDEFORM', text = 'Fatty Tissue')
        layout.separator()
        layout.operator("mesh.add_thick_curve", icon= 'ROOTCURVE')




class SOFASceneProperties(bpy.types.PropertyGroup):
    """SOFA properties associated with a scene"""
    mu = bpy.props.FloatProperty(name=u"\u03bc",description="LCP parameter mu",soft_min=1e-9,soft_max=0.1,step=1e-6,default=1e-6,precision=6)
    alarmDistance = bpy.props.FloatProperty(name="Alarm Distance",description="Collision detection check distance",default=0.1,soft_min=1e-4,soft_max=0.1,step=1e-5,precision=3)
    contactDistance = bpy.props.FloatProperty(name="Contact Distance",default=0.01,soft_min=1e-5,soft_max=0.1,step=1e-5,precision=6)
    showXYZFrame = bpy.props.BoolProperty(name="Show XYZ frame",description="Show a small XYZ frame in the lower right corner in SOFA simulation",default=False)


class SOFAObjectProperties(bpy.types.PropertyGroup):
    """SOFA properties and annotations for objects"""
    template = bpy.props.EnumProperty(name="Template",default='VISUAL', items=[
        ('VISUAL', 'Visual', 'A decorative visual object that does not participate in simulation', 'SCENE', 1),
        ('VOLUMETRIC', 'Volumetric', 'A hexahedral or tetrahedral volumetric mesh', 'SNAP_VOLUME', 2),
        ('THICKSHELL', 'Thick Shell', 'An offset object as a thick shell', 'MOD_CLOTH', 3),
        ('THICKCURVE', 'Thick Curve', 'A thick curve made from a Bezier object', 'ROOTCURVE', 4),
        ('CLOTH', 'Cloth', 'A surface cloth', 'OUTLINER_OB_SURFACE', 5),
        ('COLLISION', 'Obstacle', '', 'SOLID', 6),
        ('SPHERECONSTRAINT','Sphere Constraint','', 'SURFACE_NSPHERE', 7),
        ('BOXCONSTRAINT', 'Box Constraint', '','OBJECT_DATA', 8),
        ('ATTACHCONSTRAINT', 'Spring Attachment', '', 'LINKED', 9),
        ('INSTRUMENT','Haptic Instrument','A haptically enabled surgical instrument', 'SCULPTMODE_HLT',10),
        ('INSTRUMENTPART', 'Intrument part', 'An animated part of the instrument', 'OOPS', 11),
        ('INSTRUMENTTIP','Tip of Instrument','Active part of the instrument that performs actions', 'OOPS', 12)
        ])

    # Instrument properties
    toolFunction = bpy.props.EnumProperty(name="Function",description="Interactive function of an instrument", default='GRASP',items=[
        ('GRASP', 'Grasp', 'A grasper instrument'),
        ('SUTURE','Suture', 'A grasper that can be used for suturing'),
        ('CARVE', 'Carve', 'An instrument that destroys tissue at contact')
        ])
    instrumentPart = bpy.props.EnumProperty(name="Animated Part type",default='FIXED',items=[
        ('LEFTJAW', 'Left Jaw', ''),
        ('RIGHTJAW', 'Right Jaw', ''),
        ('FIXED', 'Fixed', 'Fixed part of the tool that moves with the handle')
        ])
    proximity = bpy.props.FloatProperty(name="Proximity",description="Proximity for collision detection",min=0,default=0,max=10,step=0.01)

    # Collision detection and response
    collisionGroup = bpy.props.IntProperty(name="Collision Group",default=1,min=1,max=100,soft_max=10)
    selfCollision = bpy.props.BoolProperty(name="Self Collision",description="Object cannot go through itself when enabled",default=False)
    contactFriction = bpy.props.FloatProperty(name="Contact Friction",default=500,min=0,max=1e+5,step=100)
    contactStiffness = bpy.props.FloatProperty(name="Contact Stiffness",default=500,min=0,max=1e+5,step=100)
    
    # Elasticity
    youngModulus = bpy.props.FloatProperty(name="Stiffness (Young Modulus)",default=3000,min=1,max=1e+6,soft_min=10,step=100)
    poissonRatio = bpy.props.FloatProperty(name="Compressibility",default=0.45,min=0.0,max=0.49,step=0.01)
    rayleighStiffness = bpy.props.FloatProperty(name="Rayleigh Stiffness",default=0.0,min=0.0,max=0.49,step=0.01)
    bendingStiffness = bpy.props.FloatProperty(name="Bending Stiffness",default=3000,min=1,max=1e+6,step=100)
    damping = bpy.props.FloatProperty(name="Damping",default=0.1,min=0,max=1000,step=0.1)
    precomputeConstraints = bpy.props.BoolProperty(name='Accurate Constraints',description='Better and more accurate constraints but requires lengthy precomputation',default=False)
    
    # Attachments 
    attachStiffness = bpy.props.FloatProperty(name="Attach Stiffness",default=10000,min=1,max=1e+6,soft_min=10,step=100)
    alwaysMatchForObject1 = bpy.props.BoolProperty(name='Always Match for First Object',default=False)
    alwaysMatchForObject2 = bpy.props.BoolProperty(name='Always Match for Second Object',default=False)
    object1 = bpy.props.StringProperty(name='First Object', description='Name of the first object in the attachment')
    object2 = bpy.props.StringProperty(name='Second Object', description='Name of the second object in the attachment')

    # Interactive features    
    carvable = bpy.props.BoolProperty(name='Carvable',description='Allow the object be interactively carved by mouse or a carving tool',default=False)
    suture = bpy.props.BoolProperty(name='Interactive',description='Allow the object to be interactively manipulated by the haptic tools',default=True)
    
    # 
    texture3d = bpy.props.StringProperty(name='3D Texture',description='Filename of the 3D texture')
    thickness = bpy.props.FloatProperty(name='Thickness',description='Thickness of the shell', default=0.1,min=0.001,max=1,step=0.01)
    layerCount = bpy.props.IntProperty(name='Layer Count', description='Number of layers in the thick shell',default=1,min=1,max=10)
        
class HapticProperties(bpy.types.PropertyGroup):
    scale = bpy.props.FloatProperty(name='Workspace Scale',description='Scaling applied to the workspace box of the haptic',default=300,min=1,max=10000,step=10)
    forceScale = bpy.props.FloatProperty(name='Force-feedback Scale',description='Scaling applied to force feedback',default=0.03,min=0,max=10000,soft_max=1)
    forceFeedback = bpy.props.BoolProperty(name='Force-feedback enabled',description='Enable force-feedback for this haptic device',default=False)
    deviceName = bpy.props.StringProperty(name='Device Name',description='Name of the haptic device name as registered in the Geomagic Touch Setup application')
    
