
def writeln(out, level, text):
    for i in range(level):
        out.write('  ')
    out.write(text)
    out.write('\n')

def writeNode(out, n, level, parent = None, serial = 0):
    """
    Write the element tree node n to the out file
    level is used to determine the indentation level
    serial is the number of the current node
    """
    name = repr(n.get('name', ''))
    if n.tag == 'Node':
        if level == 0:
            var = 'root'
            writeln(out, level, 'root = sofa.simulation:newGraph({})'.format(name))
        else:
            var = "node{}".format(serial)
            writeln(out,level, '{} = {}:createChild({})'.format(var, parent, name))
        for a in n.attrib:
            if a != 'name':
                writeln(out,level,  '{}.{} = {}'.format(var, a, repr(n.get(a))))
    else:
        # Exporting an object
        var = "{}{}".format(n.tag, serial)
        
        # String attributes go in the newObject call
        string_attributes = ""
        for a in n.attrib:
            if isinstance(n.get(a), str):
                string_attributes = string_attributes + ', {} = {}'.format(a,repr(n.get(a)))
        writeln(out,level,'{} = {}:newObject{{ {} {} }}'.format(var, parent, repr(n.tag), string_attributes)) 
        
        # Non-string attributes must be set via __newindex
        for a in n.attrib:
            if not isinstance(n.get(a),str):
                writeln(out,level,  '{}.{} = {}'.format(var, a, repr(n.get(a))))

    # Output the children of the node
    for c in n:
        serial = writeNode(out, c, level+1, var, serial+1)
    
    return serial

def writeElementTreeToLua(root, filepath):
    out = open(filepath, "w")

    writeln(out, 0, "-- SOFA SaLua scene --")
    writeNode(out, root, 0)
    writeln(out, 0, "return root")
    out.close()
