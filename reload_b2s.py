# Use this to reload Blender2SOFA inside blender
# To use:
# Open 'Scripting' mode in Blender
# In the text editor area click 'Open'
# Select this file
# Hit 'Run Script' to reload Blender2SOFA
import imp
import blender2sofa

blender2sofa.unregister()

blender2sofa = imp.reload(blender2sofa)
blender2sofa.types = imp.reload(blender2sofa.types)
blender2sofa.io_msh = imp.reload(blender2sofa.io_msh)
blender2sofa.ui = imp.reload(blender2sofa.ui)
blender2sofa.conn_tiss = imp.reload(blender2sofa.conn_tiss)
blender2sofa.thick_curve = imp.reload(blender2sofa.thick_curve)
blender2sofa.export = imp.reload(blender2sofa.export)
blender2sofa.fattytissue = imp.reload(blender2sofa.fattytissue)
blender2sofa.runsofa = imp.reload(blender2sofa.runsofa)
blender2sofa.preferences = imp.reload(blender2sofa.preferences)

blender2sofa.register()
