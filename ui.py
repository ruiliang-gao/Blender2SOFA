import bpy

SOFA_SCENE_PROPERTIES = {
    'mu': { 'default' : 0.001, 'min' : 0.001, 'max' : 0.1, 'step' : 0.001, 'precision': 3 },
    'alarmDistance': { 'default': 0.1, 'min' : 0.0001, 'max' : 1.0, 'step' : 0.001, 'precision': 3},
    'contactDistance': { 'default': 0.01, 'min': 0.0001, 'max' : 1.0, 'step': 0.001, 'precision': 3},
    'includes': { 'default': '' },
    'displayFlags': { 'default': 'showVisualModels'},
}

OBJECT_MAP = {
    'SOFT_BODY': ( "Soft Body", 'MOD_SOFT', {'resX':10, 'resY':10, 'resZ':10, 'youngModulus':300, 'poissonRatio':0.45, 'rayleighStiffness':0, 'contactFriction':0.01, 'contactStiffness':500, 'collisionGroup':1}), 
    'CLOTH' : ("Cloth",'OUTLINER_OB_SURFACE', {'youngModulus':300, 'poissonRatio': { 'default': 0.45, 'min': 0.0, 'max' : 0.5, 'step': 0.001 }, 'bendingStiffness':300, 'stretchDamping':0.1, 'bendingDamping':0.1, 'collisionGroup':1}),
    'COLLISION': ("Obstacle",'SOLID', {'collisionGroup':1}),
    'CONNECTIVETISSUE': ("Connective Tissue",'LINKED', {'stiffness':1000, 'object1':'', 'object2':'', 'alwaysMatchFor': { 'default': 0, 'min': 0, 'max' : 2, 'step': 1, 'description': 'Always find springs for object x where (0 = None, 1 = Obj1, 2 = Obj2)' }, '3dtexture': '', 'selfCollision': False, 'precomputeConstraints' : False, 'carvable': False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'suture': False}),     
    'ATTACHCONSTRAINT': ("Attach Constraint",'LINKED', {'stiffness':1000, 'object1':'', 'object2':'', 'alwaysMatchFor': { 'default': 0, 'min': 0, 'max' : 2, 'step': 1, 'description': 'Always find springs for object x where (0 = None, 1 = Obj1, 2 = Obj2)' }}), 
    'SPHERECONSTRAINT': ("Sphere Constraint",'SURFACE_NSPHERE', {}),
    'VOLUMETRIC': ("Volumetic",'SNAP_VOLUME', { '3dtexture': '', 'selfCollision': False, 'precomputeConstraints' : False, 'carvable': False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'suture': False} ), 
    'THICKSHELL': ("Thick Shell",'MOD_CLOTH', { 'selfCollision': False, 'precomputeConstraints' : False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500, 'collisionGroup':1, 'thickness': 0.1 , 'suture': False, 'layerCount': 1 } ), 
    'HAPTIC':("Haptic",'SCULPTMODE_HLT', {'scale':300, 'forceScale': 0.1, 'forceFeedback' : False, 'toolFunction': 'Grasp', 'deviceName': '', 'collisionGroup':1}), 
    'INSTRUMENT':("Instrument", 'SCULPTMODE_HLT', { 'collisionGroup':1, 'function': 'suture' }), 
    'INSTRUMENTPART': ("Instrument Part",'OOPS',{'index':{'default':3,'min':1,'max':3,'step':1}}),
    'INSTRUMENTTIP': ("Instrument Tip",'OOPS',{}),
    'THICKCURVE': ("Thick Curve", 'ROOTCURVE', { 'thickness': 0.1 }),
    'RIGID':("Rigid",'MESH_ICOSPHERE', {'collisionGroup':1})
}

OBJECT_LIST = [ 
  'SOFT_BODY', 'VOLUMETRIC', 'THICKSHELL', 'THICKCURVE', 'CONNECTIVETISSUE',
  'CLOTH', 'COLLISION', 'RIGID',
  'HAPTIC',  'INSTRUMENT', 'INSTRUMENTPART', 'INSTRUMENTTIP',
  'SPHERECONSTRAINT', 'ATTACHCONSTRAINT'
  ]


class MakeSofaSceneOperator(bpy.types.Operator):
    bl_label = "Make SOFA scene"
    bl_idname = "sofa.makescene"

    @classmethod
    def poll(cls,context):
        return context.scene != None

    def execute(self,context):
        s = context.scene
        s['sofa']=True
        s["_RNA_UI"] = s.get("_RNA_UI", {})
        for prop in SOFA_SCENE_PROPERTIES:
            val = SOFA_SCENE_PROPERTIES[prop]
            s["_RNA_UI"][prop]= val
            s[prop] = val['default']
        return { 'FINISHED' }


class SofaPropertyPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "SOFA Properties"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    
        
    
    def draw(self, context):
        layout = self.layout

        obj = context.object

        row = layout.row()
        row.operator("scene.runsofa", icon='PLAY')
        layout.operator("mesh.construct_con_tissue", icon='OUTLINER_OB_META')
        
        if obj != None: 
            layout.separator()
            antype = obj.get("annotated_type")
            
            c = layout.column_flow(align=True, columns=1)
            c.label(text="Object Kind", icon='OBJECT_DATAMODE')
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
        
        s = context.scene
        if s != None:
            layout.separator()
            if s.get('sofa') != None:
                c = layout.column_flow(align=True, columns=1)
                c.label(text="Scene Properties", icon='SCENE_DATA')
                p = SOFA_SCENE_PROPERTIES 
                l = list(p.keys()); l.sort()
                for e in l:
                  if type(p[e]['default']) != str:
                    c.prop(s, '["'+ e + '"]')
                for e in l:
                  if type(p[e]['default']) == str:
                    c.label(text=e + ':')
                    c.prop(s, '["'+ e +'"]', '')

            else:
                layout.operator("sofa.makescene")

        
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
        if( type==self.kind):
            #delete the type and related properties
            del o["annotated_type"]
            (t,i,p) = OBJECT_MAP[self.kind]
            for e in p:
                if o.get(e):
                        del o[e]
                elif o.get(e) == 0 :
                    del o[e]
                elif o.get(e) == "" :
                    del o[e]
        #if the object type is other types
        elif (type != None):
            #delete previous properties
            (text,icon,properties) = OBJECT_MAP[type]
            for prop in properties:
                if o.get(prop):
                        del o[prop]
                elif o.get(prop) == 0 :
                    del o[prop]
                elif o.get(prop) == "" :
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
        #if the object doesn't have any type
        else :
            #assign the type and related properties for it
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
        
def register():
    bpy.utils.register_class(SetAnnotatedTypeButton)
    bpy.utils.register_class(SofaPropertyPanel)
    bpy.utils.register_class(MakeSofaSceneOperator)

def unregister():
    bpy.utils.unregister_class(SetAnnotatedTypeButton)
    bpy.utils.unregister_class(SofaPropertyPanel)
    bpy.utils.unregister_class(MakeSofaSceneOperator)
