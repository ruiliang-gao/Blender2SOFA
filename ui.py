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

        row = layout.row()
        if(obj.get("annotated_type") =='SOFT_BODY'):
            row.operator("my.button", text="Soft Body", icon='X').number = 1
        else:
            row.operator("my.button", text="Soft Body", icon='MOD_SOFT').number = 1
        
        row = layout.row()
        if(obj.get("annotated_type") =='CLOTH'):
            row.operator("my.button", text="Cloth", icon='X').number = 2
        else:
            row.operator("my.button", text="Cloth", icon='MOD_CLOTH').number = 2
        
        row = layout.row()
        if(obj.get("annotated_type") =='COLLISION'):
            row.operator("my.button", text="Obstacle", icon='X').number = 3
        else:
            row.operator("my.button", text="Obstacle", icon='MOD_EDGESPLIT').number = 3
        
        row = layout.row()
        if(obj.get("annotated_type") =='ATTACHCONSTRAINT'):
            row.operator("my.button", text="Attach Constraint", icon='X').number = 4
        else:
            row.operator("my.button", text="Attach Constraint", icon='CONSTRAINT_DATA').number = 4
        
        row = layout.row()
        if(obj.get("annotated_type") =='SPHERECONSTRAINT'):
            row.operator("my.button", text="Sphere Constraint", icon='X').number = 5
        else:
            row.operator("my.button", text="Sphere Constraint", icon='CONSTRAINT').number = 5
        
        row = layout.row()
        if(obj.get("annotated_type") =='VOLUMETRIC'):
            row.operator("my.button", text="Volumetic", icon='X').number = 6
        else:
            row.operator("my.button", text="Volumetic", icon='SNAP_VOLUME').number = 6
        
        row = layout.row()
        if(obj.get("annotated_type") =='HAPTIC'):
            row.operator("my.button", text="Haptic", icon='X').number = 7
        else:
            row.operator("my.button", text="Haptic", icon='MODIFIER').number = 7
        
        row = layout.row()
        if(obj.get("annotated_type") =='RIGID'):
            row.operator("my.button", text="Rigid", icon='X').number = 8
        else:
            row.operator("my.button", text="Rigid", icon='MESH_ICOSPHERE').number = 8
        
#   Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.button"
    bl_label = "Button"
    number = bpy.props.IntProperty()
    loc = bpy.props.StringProperty()
 
    def execute(self, context):
        if self.loc:
            words = self.loc.split()
            self.number = int(words[1])
        print("Button %d" % (self.number))
        
        o = bpy.context.object
        
        if (self.number == 1 ):
            if(o.get("annotated_type") =='SOFT_BODY'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'SOFT_BODY'
        elif (self.number == 2):
            if(o.get("annotated_type") =='CLOTH'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'CLOTH'
        elif (self.number == 3):
            if(o.get("annotated_type") =='COLLISION'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'COLLISION'
        elif (self.number == 4):
            if(o.get("annotated_type") =='ATTACHCONSTRAINT'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'ATTACHCONSTRAINT'
        elif (self.number == 5):
            if(o.get("annotated_type") =='SPHERECONSTRAINT'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'SPHERECONSTRAINT'
        elif (self.number == 6):
            if(o.get("annotated_type") =='VOLUMETRIC'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'VOLUMETRIC'
        elif (self.number == 7):
            if(o.get("annotated_type") =='HAPTIC'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'HAPTIC'
        elif (self.number == 8):
            if(o.get("annotated_type") =='RIGID'):
                del o["annotated_type"]
            else:
                o['annotated_type'] = 'RIGID'
            
        return{'FINISHED'}    

bpy.utils.register_class(OBJECT_OT_Button)
bpy.utils.register_class(SofaPropertyPanel)