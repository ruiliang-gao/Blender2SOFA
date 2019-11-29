import bpy
import os
from .export import exportScene,writeNodesToFile,FILEFORMATS, ExportOptions, ExportException
from subprocess import Popen
from tempfile import mktemp
import sys
from bpy.props import StringProperty, BoolProperty, EnumProperty

from pprint import pprint

def updateFileFormat(self, context):
    base, ext = os.path.splitext(self.filepath)
    self.filepath = base + self.file_format

class RunSofaOperator(bpy.types.Operator):
    bl_idname = "scene.runsofa"
    bl_label = "Run Simulation in Sofa"
    bl_options = { 'REGISTER', 'UNDO' }
    file_format : EnumProperty(name = "File format",
    items = FILEFORMATS, update = updateFileFormat, default='.scn')

    filepath : StringProperty(name = "Filepath")

    @classmethod
    def poll(cls, context):
        return context.scene is not None

    def invoke(self, context, event):
        ext = self.file_format
        if bpy.data.filepath == '':
            self.filepath = mktemp(suffix=ext)
        else:
            self.filepath = bpy.data.filepath + ext
        return self.execute(context)

    def execute(self, context):
        self.report({'INFO'}, "Exporting to %s" % self.filepath)
        try:
            opt = ExportOptions()
            opt.isolate_geometry = False
            opt.scene = context.scene
            opt.separate = False
            opt.selection_only = False
            opt.directory = os.path.dirname(self.filepath)
            opt.file_format = self.file_format
            print("HELLO2")
            print(self.filepath)
            opt.pref = context.preferences.addons[__package__].preferences
            print("HELLOO")
            root = exportScene(opt)
            print("HELLOOO")
            writeNodesToFile(root,self.filepath, opt)
            print("ASDF")
            if sys.platform == 'linux':
                Popen(['xterm', '-e', 'runSofa', self.filepath])
            else: # if sys.platform == 'windows':
                print("HELLO1")
                Popen(self.filepath,shell=True)
            return {'FINISHED'}
        except ExportException as et:
            self.report({'ERROR'}, "Export failed: %s" % et.message)
            return { 'CANCELLED' }
