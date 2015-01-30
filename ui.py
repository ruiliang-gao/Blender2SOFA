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
        row.operator("my.button", text="Soft Body").number = 1
        
        row = layout.row()
        row.operator("my.button", text="Cloth").number = 2
        
        row = layout.row()
        row.operator("my.button", text="Obstacle").number = 3
        
        row = layout.row()
        row.operator("my.button", text="Attach Constraint").number = 4
        
        row = layout.row()
        row.operator("my.button", text="Sphere Constraint").number = 5
        
        row = layout.row()
        row.operator("my.button", text="Volumetic").number = 6
        
        row = layout.row()
        row.operator("my.button", text="Haptic").number = 7
        
        row = layout.row()
        row.operator("my.button", text="Rigid").number = 8
        
#   Button
class OBJECT_OT_Button(bpy.types.Operator):
    bl_idname = "my.button"
    bl_label = "Button"
    number = bpy.props.IntProperty()
    row = bpy.props.IntProperty()
    loc = bpy.props.StringProperty()
 
    def execute(self, context):
        if self.loc:
            words = self.loc.split()
            self.number = int(words[1])
        print("Row %d button %d" % (self.row, self.number))
        
        o = bpy.context.object
        
        if (self.number == 1 ):
            o['annotated_type'] = 'SOFT_BODY'
        elif (self.number == 2):
            o['annotated_type'] = 'CLOTH'
        elif (self.number == 3):
            o['annotated_type'] = 'COLLISION'
        elif (self.number == 4):
            o['annotated_type'] = 'ATTACHCONSTRAINT'
        elif (self.number == 5):
            o['annotated_type'] = 'SPHERECONSTRAINT'
        elif (self.number == 6):
            o['annotated_type'] = 'VOLUMETRIC'
        elif (self.number == 7):
            o['annotated_type'] = 'HAPTIC'
        elif (self.number == 8):
            o['annotated_type'] = 'RIGID'
            
        return{'FINISHED'}    

bpy.utils.register_class(OBJECT_OT_Button)
bpy.utils.register_class(SofaPropertyPanel)