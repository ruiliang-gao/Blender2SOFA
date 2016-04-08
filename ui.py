import bpy


OBJECT_MAP = {
    'SOFT_BODY': ( "Soft Body", 'MOD_SOFT', {'resX':10, 'resY':10, 'resZ':10, 'youngModulus':300, 'poissonRatio':0.45, 'rayleighStiffness':0, 'contactFriction':0.01, 'contactStiffness':500, 'collisionGroup':1}),
    'CLOTH' : ("Cloth",'OUTLINER_OB_SURFACE', {'youngModulus':300, 'poissonRatio': { 'default': 0.45, 'min': 0.0, 'max' : 0.5, 'step': 0.001 }, 'bendingStiffness':300, 'stretchDamping':0.1, 'bendingDamping':0.1, 'collisionGroup':1}),
    'COLLISION': ("Obstacle",'SOLID', {'collisionGroup':1}),
    'CONNECTIVETISSUE': ("Connective Tissue",'LINKED', {'attach_stiffness': 10000, 'topObject':'', 'botObject':'', 'alwaysMatchFor': { 'default': 0, 'min': 0, 'max' : 2, 'step': 1, 'description': 'Always find springs for object x where (0 = None, 1 = Obj1, 2 = Obj2)' }, '3dtexture': '', 'selfCollision': False, 'precomputeConstraints' : False, 'carvable': False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'suture': False}),
    'ATTACHCONSTRAINT': ("Spring Attachment",'LINKED', {'stiffness':1000, 'object1':'', 'object2':'', 'alwaysMatchFor': { 'default': 0, 'min': 0, 'max' : 2, 'step': 1, 'description': 'Always find springs for object x where (0 = None, 1 = Obj1, 2 = Obj2)' }}),
    'SPHERECONSTRAINT': ("Sphere Constraint",'SURFACE_NSPHERE', {}),
    'VOLUMETRIC': ("Volumetic",'SNAP_VOLUME', { '3dtexture': '', 'selfCollision': False, 'precomputeConstraints' : False, 'carvable': False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'suture': False} ),
    'THICKSHELL': ("Thick Shell",'MOD_CLOTH', { 'degree': { 'default': 1, 'min': 1, 'max': 3, 'step': 1 }, 'selfCollision': False, 'precomputeConstraints' : False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'thickness': 0.1 , 'suture': False, 'layerCount': 1 } ),
    'HAPTIC':("Haptic",'SCULPTMODE_HLT', {'scale':300, 'forceScale': 0.001, 'forceFeedback' : False, 'deviceName': ''}),
    'THICKCURVE': ("Thick Curve", 'ROOTCURVE', { 'thickness': 0.1 }),
    'RIGID':("Rigid",'MESH_ICOSPHERE', {'collisionGroup':1})
}

OBJECT_LIST = [
  'SOFT_BODY', 'VOLUMETRIC', 'THICKSHELL', 'THICKCURVE', 'CONNECTIVETISSUE',
  'CLOTH', 'COLLISION', 'RIGID',
  'HAPTIC',
  'SPHERECONSTRAINT', 'ATTACHCONSTRAINT'
  ]


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
        p = context.object.sofaprops
        layout = self.layout
        layout.prop(p, 'template', text='')

        t = p.template
        c = layout.column_flow(align=True,columns=1)
        if t == 'INSTRUMENT':
            c.prop(p, 'toolfunction')
        elif t == 'INSTRUMENTPART':
            c.prop(p, 'instrument_part_type')





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

        c = layout.column()
        c.operator("scene.runsofa", icon='PLAY')
        c.operator("mesh.construct_con_tissue", icon='OUTLINER_OB_META')
        c.operator("mesh.construct_hex_rod", icon='MOD_MESHDEFORM')
        c.operator("mesh.construct_fatty_tissue", icon='MOD_MESHDEFORM')


        obj = context.object
        if obj != None:
            layout.separator()
            antype = obj.get("annotated_type")

            c = layout.column_flow(align=True, columns=1)
            c.label(text="SOFA annotation:", icon='OBJECT_DATAMODE')
            for n in OBJECT_LIST:
                (t,i,p) = OBJECT_MAP[n]
                if(antype == n):
                    c.operator("tips.setannotatedtype", text=t, icon='X').kind = n
                else:
                    c.operator("tips.setannotatedtype", text=t, icon=i).kind = n

            if antype != None:
                layout.separator()
                c = layout.column_flow(align=True, columns = 1)
                c.label(text="Object Properties", icon='OBJECT_DATAMODE')
                (t,i,p) = OBJECT_MAP[antype]
                l = list(p.keys()); l.sort()
                for e in l:
                  if type(p[e]) != str:
                    c.prop(obj, '["'+ e + '"]')
                for e in l:
                  if type(p[e]) == str:
                    c.label(text=e + ':')
                    c.prop(obj, '["'+ e +'"]', '')

        layout.separator()


#   Button
class SetAnnotatedTypeButton(bpy.types.Operator):
    bl_idname = "tips.setannotatedtype"
    bl_label = "Set Annotated Type"
    number = bpy.props.IntProperty()
    kind = bpy.props.StringProperty()

    def execute(self, context):

        o = context.object
        type = o.get("annotated_type")
        #if the object type is already this kind of type
        if type==self.kind:
            #delete the type and related properties
            del o["annotated_type"]
            (t,i,p) = OBJECT_MAP[self.kind]
            for e in p:
                if o.get(e) != None:
                        del o[e]
        #if the object type is utilsother types
        else:
          if type != None:
            #delete previous properties
            (text,icon,properties) = OBJECT_MAP[type]
            for prop in properties:
                if o.get(prop) != None:
                        del o[prop]
            
          #set the current type and related properties
          o['annotated_type'] = self.kind
          (t,i,p) = OBJECT_MAP[self.kind]
          o["_RNA_UI"] = o.get("_RNA_UI", {})
          for e in p:
              if isinstance(p[e], dict):
                  o["_RNA_UI"][e] = p[e]
                  o[e] = p[e]['default']
              else:
                  o[e] = p[e]

        return{'FINISHED'}


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
        ('INSTRUMENT','Haptic Instrument','A haptically enabled surgical instrument', 'SCULPTMODE_HLT', 2),
        ('INSTRUMENTPART', 'Intrument part', 'An animated part of the instrument', 'OOPS', 3),
        ('INSTRUMENTTIP','Tip of Instrument','Active part of the instrument that performs actions', 'OOPS', 4)
        ])

    toolfunction = bpy.props.EnumProperty(name="Function",description="Interactive function of an instrument", default='GRASP',items=[
        ('GRASP', 'Grasp', 'A grasper instrument'),
        ('SUTURE','Suture', 'A grasper that can be used for suturing'),
        ('CARVE', 'Carve', 'An instrument that destroys tissue at contact')
        ])

    instrument_part_type = bpy.props.EnumProperty(name="Animated Part type",default='FIXED',items=[
        ('LEFTJAW', 'Left Jaw', ''),
        ('RIGHTJAW', 'Right Jaw', ''),
        ('FIXED', 'Fixed', 'Fixed part of the tool that moves with the handle')
        ])
