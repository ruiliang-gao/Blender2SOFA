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
        row.label(text="SOFA Properties", icon='WORLD_DATA')

        l = [ ('SOFT_BODY', "Soft Body", 'MOD_SOFT'), 
                ('CLOTH',"Cloth",'MOD_CLOTH'),('COLLISION',"Obstacle",'MOD_EDGESPLIT'),
            ('ATTACHCONSTRAINT',"Attach Constraint",'CONSTRAINT_DATA'), ('SPHERECONSTRAINT',"Sphere Constraint",'CONSTRAINT'),
            ('VOLUMETRIC',"Volumetic",'SNAP_VOLUME'), ('HAPTIC',"Haptic",'MODIFIER'), ('RIGID',"Rigid",'MESH_ICOSPHERE')]
            
        for index,(n,t,i) in enumerate(l) :
            
            row = layout.row()
            if(obj.get("annotated_type") == n):
                row.operator("my.button", text=t, icon='X').kind = n
                
            else:
                row.operator("my.button", text=t, icon=i).kind = n
        
#   Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.button"
    bl_label = "Button"
    number = bpy.props.IntProperty()
    kind = bpy.props.StringProperty()
 
    def execute(self, context):
        
        
        o = bpy.context.object
       
        if(o.get("annotated_type") ==self.kind):
            del o["annotated_type"]
        else:
            o['annotated_type'] = self.kind
      
        return{'FINISHED'}    

bpy.utils.register_class(OBJECT_OT_Button)
bpy.utils.register_class(SofaPropertyPanel)