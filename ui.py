import bpy

SOFA_SCENE_PROPERTIES = {
    'mu': { 'default' : 0.001, 'min' : 0.001, 'max' : 0.1, 'step' : 0.001, 'precision': 3 },
    'alarmDistance': { 'default': 0.1, 'min' : 0.0001, 'max' : 1.0, 'step' : 0.001, 'precision': 3},
    'contactDistance': { 'default': 0.01, 'min': 0.0001, 'max' : 1.0, 'step': 0.001, 'precision': 3},
    'includes': { 'default': '' },
    'displayFlags': { 'default': 'visualModels'},
}

OBJECT_LIST = {
    'SOFT_BODY': ( "Soft Body", 'MOD_SOFT', {'resX':10, 'resY':10, 'resZ':10, 'youngModulus':300, 'poissonRatio':0.45, 'rayleighStiffness':0, 'contactFriction':0.01, 'contactStiffness':500}), 
    'CLOTH' : ("Cloth",'MOD_CLOTH', {'youngModulus':300, 'poissonRatio': { 'default': 0.45, 'min': 0.0, 'max' : 0.5, 'step': 0.001 }, 'bendingStiffness':300, 'stretchDamping':0.1, 'bendingDamping':0.1}),
    'COLLISION': ("Obstacle",'MOD_EDGESPLIT', {}),
    'ATTACHCONSTRAINT': ("Attach Constraint",'CONSTRAINT_DATA', {'stiffness':1000}), 
    'SPHERECONSTRAINT': ("Sphere Constraint",'CONSTRAINT', {}),
    'VOLUMETRIC': ("Volumetic",'SNAP_VOLUME', { 'carvable': False, 'youngModulus': 300 , 'poissonRatio':0.45, 'damping': 0.1, 'contactFriction': 0.01, 'contactStiffness':500} ), 
    'HAPTIC':("Haptic",'MODIFIER', {'scale':300, 'forceScale': 0.1, 'forceFeedback' : False, 'toolFunction': 'Grasp'}), 
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
        row.operator("scene.runsofa")

        if obj != None: 
            row = layout.row()
            row.label(text="Object Properties", icon='OBJECT_DATAMODE')
            antype = obj.get("annotated_type")
            
            for n in OBJECT_LIST:
                (t,i,p) = OBJECT_LIST[n]
                row = layout.row()
                if(antype == n):
                    row.operator("tips.setannotatedtype", text=t, icon='X').kind = n
                    for e in p:
                        row = layout.row()
                        row.prop(obj, '["'+ e + '"]')
                
                else:
                    row.operator("tips.setannotatedtype", text=t, icon=i).kind = n
                    
                    

        s = context.scene
        if s != None:        

            if s.get('sofa') != None:
                row = layout.row()
                row.label(text="Scene Properties", icon='SCENE_DATA')
        
                for i in SOFA_SCENE_PROPERTIES:
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

if __name__ == "__main__":
    register()
