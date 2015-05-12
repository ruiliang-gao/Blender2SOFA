from ctypes import *
import numpy as np
import numpy.ctypeslib as npc 



p_int = POINTER(c_int)
c_real = c_double
p_real = POINTER(c_real)
c_void = None

class TriangleMesh(Structure):
    _fields_ = [
        ('vertex', p_real),
        ('vertexCount', c_int),
        ('triangle', p_int),
        ('triangleCount', c_int)
    ]

    def __init__(self):
        self.vertex = None
        self.triangle = None
        self.vertexCount = 0
        self.triangleCount = 0

    def set_vertices(self, v):
        if isinstance(v, np.ndarray) and v.shape[1] == 3:
            self.vertex = npc.as_ctypes(v.flatten())
            self.vertexCount = len(v)
        else:
            raise ArgumentError("numpy array expected")

    def get_vertices(self):
        return npc.as_array(self.vertex, shape=(self.vertexCount,3))

    def set_triangles(self, t):
        if isinstance(t, np.ndarray) and t.shape[1] == 3:
            self.triangle = npc.as_ctypes(t.flatten())
            self.triangleCount = len(t)
        else:
            raise ArgumentError("numpy array expected")

    def get_triangles(self):
        return npc.as_array(self.triangle, shape=(self.triangleCount,3))

    vertices = property(get_vertices, set_vertices)
    triangles = property(get_triangles, set_triangles)

class TetrahedralMesh(Structure):
    _fields_ = [
        ('vertex', p_real),
        ('vertexCount', c_int),
        ('triangle', p_int),
        ('triangleCount', c_int),
        ('tetrahedron', p_int),
        ('tetrahedronCount', c_int)
    ]

    def __init__(self):
        self.vertex = None
        self.triangle = None
        self.tetrahedron = None
        self.vertexCount = 0
        self.triangleCount = 0
        self.tetrahedronCount = 0

    def set_vertices(self, v):
        if isinstance(v, np.ndarray) and v.shape[1] == 3:
            self.vertex = npc.as_ctypes(v.flatten())
            self.vertexCount = len(v)
        else:
            raise ArgumentError("numpy array expected")

    def get_vertices(self):
        return npc.as_array(self.vertex, shape=(self.vertexCount,3))

    def set_triangles(self, t):
        if isinstance(t, np.ndarray) and t.shape[1] == 3:
            self.triangle = npc.as_ctypes(t.flatten())
            self.triangleCount = len(t)
        else:
            raise ArgumentError("numpy array expected")

    def get_triangles(self):
        return npc.as_array(self.triangle, shape=(self.triangleCount,3))

    def set_tetrahedra(self, t):
        if isinstance(t, np.ndarray) and t.shape[1] == 4:
            self.tetrahedron = npc.as_ctypes(t.flatten())
            self.tetrahedronCount = len(t)
        else:
            raise ArgumentError("numpy array expected")

    def get_tetrahedra(self):
        return npc.as_array(self.tetrahedron, shape=(self.tetrahedronCount,4))


    vertices = property(get_vertices, set_vertices)
    triangles = property(get_triangles, set_triangles)
    tetrahedra = property(get_tetrahedra,set_tetrahedra)




from os import path, environ
from sys import platform
if platform == 'win32':
    library_file = 'cgaltetrahedralize.dll'
elif platform == 'unix':
    library_file = 'libcgaltetrahedralize.so'
else:
    raise RuntimeError("Platform not supported");

environ['PATH'] = environ['PATH'] + ';' + path.join(path.dirname(__file__));
libcgaltetrahedralize = cdll.LoadLibrary(library_file)

tetrahedralize = libcgaltetrahedralize.tetrahedralize
tetrahedralize.argtypes = [ POINTER(TriangleMesh), POINTER(TetrahedralMesh) ]

