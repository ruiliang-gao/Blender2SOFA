import bpy
import xml.etree.ElementTree as ET
import os
from mathutils import Vector

C = bpy.context

def vector_to_string(v):
    t = ""
    for i in v :
        t += str(i) + " "
    return t


def write_some_data(context, filepath, use_some_setting):
    dir = os.path.dirname(filepath)
    
    bpy.ops.object.select_all(False)
    
    f = open(filepath, 'wb')
    root= ET.Element("Node")
    root.set("name", "root")
    for i in C.scene.objects: 
        if i.type == "MESH":
            o =  ET.Element("Node")
            o.set("name", i.name)
            i.rotation_mode = "ZYX"
            oglmodel = ET.SubElement(o, "OglModel")
            oglmodel.set("name", i.name)
            oglmodel.set("fileMesh",  i.name + ".obj")
            oglmodel.set("translation", vector_to_string(i.location))
            oglmodel.set("rotation", vector_to_string(i.rotation_euler))
    
            i.select = True
            bpy.ops.export_scene.obj(filepath = os.path.join(dir, i.name + ".obj"), use_selection=True, axis_forward="Y", axis_up="Z")
            i.select = False
            
            root.append(o)
        elif i.type == "LAMP":
            o = ET.Element("SpotLight")
            i.rotation_mode = "QUATERNION"
            o.set("position", vector_to_string(i.location))
            o.set("color", vector_to_string(i.color))
            direction = i.rotation_quaternion * Vector((0,0,-1))
            o.set("direction",vector_to_string(direction))
            
            root.append(o)
            
    ET.ElementTree(root).write(f)
    f.close()

    return {'FINISHED'}


# ExportHelper is a helper class, defines filename and
# invoke() function which calls the file selector.
from bpy_extras.io_utils import ExportHelper
from bpy.props import StringProperty, BoolProperty, EnumProperty
from bpy.types import Operator


class ExportToSofa(Operator, ExportHelper):
    """Export to Sofa XML scene format"""
    bl_idname = "export.tosofa"  # important since its how bpy.ops.import_test.some_data is constructed
    bl_label = "Export To Sofa"

    # ExportHelper mixin class uses this
    filename_ext = ".scn"

    filter_glob = StringProperty(
            default="*.scn",
            options={'HIDDEN'},
            )

    # List of operator properties, the attributes will be assigned
    # to the class instance from the operator settings before calling.
    use_setting = BoolProperty(
            name="Example Boolean",
            description="Example Tooltip",
            default=True,
            )

    type = EnumProperty(
            name="Example Enum",
            description="Choose between two items",
            items=(('OPT_A', "First Option", "Description one"),
                   ('OPT_B', "Second Option", "Description two")),
            default='OPT_A',
            )

    def execute(self, context):
        return write_some_data(context, self.filepath, self.use_setting)


# Only needed if you want to add into a dynamic menu
def menu_func_export(self, context):
    self.layout.operator(ExportToSofa.bl_idname, text="To Sofa XML Scene")


def register():
    bpy.utils.register_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.append(menu_func_export)


def unregister():
    bpy.utils.unregister_class(ExportToSofa)
    bpy.types.INFO_MT_file_export.remove(menu_func_export)


if __name__ == "__main__":
    register()

    # test call
    bpy.ops.export.tosofa('INVOKE_DEFAULT')
