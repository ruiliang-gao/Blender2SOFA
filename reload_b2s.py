import imp
import blender2sofa

blender2sofa.unregister()

blender2sofa = imp.reload(blender2sofa)
blender2sofa.ui = imp.reload(blender2sofa.ui)
blender2sofa.conn_tiss = imp.reload(blender2sofa.conn_tiss)
blender2sofa.hex_rod = imp.reload(blender2sofa.hex_rod)

blender2sofa.register()