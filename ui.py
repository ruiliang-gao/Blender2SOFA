import bpy

SOFA_SCENE_PROPERTIES = {
    'mu': 0.001,
    'alarmDistance': 0.1,
    'contactDistance': 0.5,
    'includes': '',
    'displayFlags': 'visualModels'
}

OBJECT_LIST = { 
    'SOFT_BODY': ( "Soft Body", 'MOD_SOFT', {}), 
    'CLOTH' : ("Cloth",'MOD_CLOTH', {}),
    'COLLISION': ("Obstacle",'MOD_EDGESPLIT', {}),
    'ATTACHCONSTRAINT': ("Attach Constraint",'CONSTRAINT_DATA', {}), 
    'SPHERECONSTRAINT': ("Sphere Constraint",'CONSTRAINT', {}),
    'VOLUMETRIC': ("Volumetic",'SNAP_VOLUME', { 'carvable': False, 'youngModulus': 3000 } ), 
    'HAPTIC':("Haptic",'MODIFIER', {}), 
    'RIGID':("Rigid",'MESH_ICOSPHERE', {})
}

class MakeSofaSceneOperator(bpy.types.Operator):
    bl_label = "Make SOFA scene"
    bl_idname = "sofa.makescene"

    @classmethod
    def poll(cls,context):
        return context.scene != None

    def execute(self,context):
        s = context.scene
        s['mu']= 6
        s['alarmDistance']= 14
        s['contactDistance']= 1991
        s['sofa'] = True
        s['includes'] = ''
        s['displayFlags'] = 'visualModels'
        return { 'FINISHED' }


class SofaPropertyPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "SOFA Properties"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    
        
    
    def draw(self, context):
        layout = self.layout

        obj = context.object

        if obj != None: 
            row = layout.row()
            row.label(text="Object Properties", icon='OBJECT_DATAMODE')
            antype = obj.get("annotated_type")

            if antype in OBJECT_LIST:
                (t,i,p) = OBJECT_LIST[antype]
                for e in p:
                    row.prop(obj, '["'+ e + '"]')

            
            for index,n in enumerate(OBJECT_LIST) :
                (t,i,p) = OBJECT_LIST[n]
                row = layout.row()
                if(antype == n):
                    row.operator("tips.setannotatedtype", text=t, icon='X').kind = n
                
                else:
                    row.operator("tips.setannotatedtype", text=t, icon=i).kind = n

        s = context.scene
        if s != None:        

            if s.get('sofa') != None:
                row = layout.row()
                row.label(text="Scene Properties", icon='SCENE_DATA')
        
                for i in [ 'mu' , 'alarmDistance', 'contactDistance' ]:
                    row = layout.row(align=True)
                    row.prop(s, '["' + i + '"]') 

            else:
                layout.row();
                layout.operator("sofa.makescene")

        
#   Button
class SetAnnotatedTypeButton(bpy.types.Operator):
    bl_idname = "tips.setannotatedtype"
    bl_label = "Set Annotated Type"
    number = bpy.props.IntProperty()
    kind = bpy.props.StringProperty()
 
    def execute(self, context):
               
        o = context.object
        
        if(o.get("annotated_type") ==self.kind):
            del o["annotated_type"]
        else:
            o['annotated_type'] = self.kind
            (t,i,p) = OBJECT_LIST[self.kind]
            for e in p:
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

if __name__ == "__main__":
    register()
