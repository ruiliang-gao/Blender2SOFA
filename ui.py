import bpy

class SofaPropertyPanel(bpy.types.Panel):
    """Creates a Panel in the Object properties window"""
    bl_label = "SOFA Properties"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'TOOLS'
    
    def draw(self, context):
        layout = self.layout

        obj = context.object

        row = layout.row()
        row.label(text="Object Properties", icon='OBJECT_DATAMODE')

        l = [ ('SOFT_BODY', "Soft Body", 'MOD_SOFT'), 
                ('CLOTH',"Cloth",'MOD_CLOTH'),('COLLISION',"Obstacle",'MOD_EDGESPLIT'),
            ('ATTACHCONSTRAINT',"Attach Constraint",'CONSTRAINT_DATA'), ('SPHERECONSTRAINT',"Sphere Constraint",'CONSTRAINT'),
            ('VOLUMETRIC',"Volumetic",'SNAP_VOLUME'), ('HAPTIC',"Haptic",'MODIFIER'), ('RIGID',"Rigid",'MESH_ICOSPHERE')]
            
        for index,(n,t,i) in enumerate(l) :
            
            row = layout.row()
            if(obj.get("annotated_type") == n):
                row.operator("tips.setannotatedtype", text=t, icon='X').kind = n
                
            else:
                row.operator("tips.setannotatedtype", text=t, icon=i).kind = n
        
        row = layout.row()
        row.label(text="Scene Properties", icon='SCENE_DATA')
                
        row = layout.row(align=True)
        row.prop(bpy.context.scene, '["mu"]')
        
        row = layout.row(align=True)
        row.prop(bpy.context.scene, '["alarmDistance"]')
        
        row = layout.row(align=True)
        row.prop(bpy.context.scene, '["constactDistance"]')
        
#   Button
class SetAnnotatedTypeButton(bpy.types.Operator):
    bl_idname = "tips.setannotatedtype"
    bl_label = "Set Annotated Type"
    number = bpy.props.IntProperty()
    kind = bpy.props.StringProperty()
 
    def execute(self, context):
        
        
        o = bpy.context.object
       
        if(o.get("annotated_type") ==self.kind):
            del o["annotated_type"]
        else:
            o['annotated_type'] = self.kind
      
        return{'FINISHED'}    

def register():
    bpy.utils.register_class(SetAnnotatedTypeButton)
    bpy.utils.register_class(SofaPropertyPanel)

def unregister():
    bpy.utils.unregister_class(SetAnnotatedTypeButton)
    bpy.utils.unregister_class(SofaPropertyPanel)

if __name__ == "__main__":
    register()
