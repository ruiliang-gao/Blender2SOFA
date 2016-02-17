import imp
import export2sofa

export2sofa.unregister()

export2sofa = imp.reload(export2sofa)
export2sofa.ui = imp.reload(export2sofa.ui)
export2sofa.conn_tiss = imp.reload(export2sofa.conn_tiss)
export2sofa.hex_rod = imp.reload(export2sofa.hex_rod)

export2sofa.register()