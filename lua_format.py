from numpy import ndarray
from io import StringIO


def writeln(out, level, text):
    for i in range(level):
        out.write('  ')
    out.write(text)
    out.write('\n')

def iterable(o):
    return hasattr(o, '__getitem__') and hasattr(o, '__len__')

def write_vector(b, v, flat):
    for i in v:
        if iterable(i):
            if not flat: b.write('{ ')
            write_vector(b, i, flat)
            if not flat: b.write(' }, ')
        else:      
            b.write(luarepr(i))
            b.write(', ')

def vector_to_string(v, flat = True):
    b = StringIO()
    b.write('{ ')
    write_vector(b, v, flat)
    b.write(' }')
    s = b.getvalue()
    b.close()
    return s

def luarepr(o):
    if isinstance(o, str):
        return repr(o)
    elif isinstance(o,int) or isinstance(o,float):
        return repr(o)
    elif isinstance(o, ndarray):
        f = o.reshape(o.size)
        return vector_to_string(f)
    elif iterable(o):
        return vector_to_string(o)
    else:
        print("Warning this representation may not be valid in Lua: " + repr(o) + " of type " + str(type(o)))
        return repr(o)

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
            writeln(out, level, 'local root = sofa.simulation:newGraph({})'.format(name))
        else:
            var = "node{}".format(serial)
            writeln(out,level, 'local {} = {}:newChild({})'.format(var, parent, name))
        for a in n.attrib:
            if a != 'name':
                writeln(out,level,  '{}.{} = {}'.format(var, a, luarepr(n.get(a))))
    elif n.tag == 'require':
        var = "{}{}".format(n.tag, serial)
        writeln(out, level, 'require("{}")({})'.format(n.get('href'), parent))
    else:
        # Exporting an object
        var = "{}{}".format(n.tag, serial)
        
        # String attributes go in the newObject call
        string_attributes = ""
        for a in n.attrib:
            if isinstance(n.get(a), str):
                string_attributes = string_attributes + ', {} = {}'.format(a,repr(n.get(a)))
        writeln(out,level,'local {} = {}:newObject{{ {} {} }}'.format(var, parent, repr(n.tag), string_attributes)) 
        
        # Non-string attributes must be set via __newindex
        for a in n.attrib:
            if not isinstance(n.get(a),str):
                writeln(out,level,  '{}.{} = {}'.format(var, a, luarepr(n.get(a))))

    # Output the children of the node
    for c in n:
        serial, v = writeNode(out, c, level+1, var, serial+1)
    
    return serial, var

def writeElementTreeToLua(root, filepath):
    out = open(filepath, "w")

    writeln(out, 0, "-- SOFA SaLua scene for {}:{}".format(root.tag,root.get('name', 'unnamed')))
    writeNode(out, root, 0)
    writeln(out, 0, "return root")
    out.close()

def writeSubTreeToLua(node, filepath):
    out = open(filepath, "w")
    writeln(out, 0, "-- SOFA SaLua subtree --")
    writeln(out, 0, "return function (parent)")
    serial, var = writeNode(out, node, 1, parent = 'parent')
    writeln(out, 1, 'return {}'.format(var))
    writeln(out, 0, "end")
    out.close()
