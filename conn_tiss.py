import bpy
import math
import numpy as np
from mathutils import Vector
from .io_msh import recalc_outer_surface

class ConnectiveTissue(bpy.types.Operator):
    bl_idname = "mesh.construct_con_tissue"
    bl_label = "Construct Connective Tissue"
    bl_options = { 'UNDO' }
    bl_description = "Works with SQUARE Grid meshes, appropriately placed between 2 objects as to avoid shrinkwrap wraparounds (overlapping vertices/degenerate tetra/hexahedra)"

    object1 = bpy.props.StringProperty(name = "Object 1", description = "Choose Object 1 here")
    object2 = bpy.props.StringProperty(name = "Object 2", description = "Choose Object 2 here")

    def invoke(self, context, event):
        return context.window_manager.invoke_props_dialog(self)

    def draw(self, context):
        layout = self.layout
        col = layout.column_flow(align=True, columns=1)
        col.prop_search(self, "object1", context.scene, "objects")
        col.prop_search(self, "object2", context.scene, "objects")

    @classmethod
    def poll(self, context):
        return (context.object is not None and context.object.type == 'MESH')

    def execute(self, context):
        construct(context, self)
        return {'FINISHED'}

    def cancel(self,context):
        return {'CANCELLED'}

def construct(context,options):

    autoDefinePlane = False
    maxDim = 1e+16
    meshType = 8 # 8 = hex, 4 = tet

    if True:
        o1 = bpy.data.objects[options.object1]  # cache the objects as dictionary indexing will change
        o2 = bpy.data.objects[options.object2]
    else:
        o1 = bpy.data.objects['a2']; o2 = bpy.data.objects['a1']

    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    if autoDefinePlane:
        bpy.ops.object.select_all(action='DESELECT')
        o1.select = True
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')
        center1t = o1.location
        center1 = Vector((center1t[0],center1t[1],center1t[2]))
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        o1.select = False
        o2.select = True
        bpy.ops.object.origin_set(type='ORIGIN_GEOMETRY')
        center2t = o2.location
        center2 = Vector((center2t[0],center2t[1],center2t[2]))
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
        o2.select = False

        footprint1 = o1.closest_point_on_mesh(center2,maxDim)
        dualfp1 = o2.closest_point_on_mesh(footprint1[0],maxDim)
        footprint2 = o2.closest_point_on_mesh(center1,maxDim)
        dualfp2 = o1.closest_point_on_mesh(footprint2[0],maxDim)

        center = footprint2[0]/2 + dualfp2[0]/2
        dim1 = min(o1.dimensions.x,o1.dimensions.y,o1.dimensions.z)
        dim2 = min(o2.dimensions.x,o2.dimensions.y,o2.dimensions.z)
        bpy.ops.mesh.primitive_grid_add(radius=min(dim1,dim2)/4, location=center)
        plane_top = context.selected_objects[0]
        bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    else:
        plane_top = context.selected_objects[0]

    # context.scene.objects.link(plane_top)
    bpy.ops.object.duplicate()
    plane_bot = context.selected_objects[0]

    # create connective tissue object (mesh to be filled in later)
    bpy.ops.object.add(type='MESH')
    bpy.ops.object.transform_apply(location=True, rotation=True, scale=True)
    ct = bpy.context.object
    #ct.name = 'ConnectiveTissue'
    ct.name = 'ct'
    ct['annotated_type'] = 'VOLUMETRIC'
    ct['carvable'] = 1
    bpy.ops.object.select_all(action='DESELECT')

    # shrinkwrap object 1
    context.scene.objects.active = plane_top
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    context.object.modifiers["Shrinkwrap"].target = o1
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

    # shrinkwrap object 2
    context.scene.objects.active = plane_bot
    bpy.ops.object.modifier_add(type='SHRINKWRAP')
    context.object.modifiers["Shrinkwrap"].use_keep_above_surface = True
    context.object.modifiers["Shrinkwrap"].target = o2
    bpy.ops.object.modifier_apply(apply_as='DATA', modifier="Shrinkwrap")

  #------ create connective tissue
    #-- create mid plane as average of top and bottom
    plane_bot.select = True
    bpy.ops.object.duplicate()
    plane_mid = context.selected_objects[0]
    for i in range(len(plane_top.data.vertices)):
        plane_mid.data.vertices[i].co = (plane_top.data.vertices[i].co + plane_bot.data.vertices[i].co)/2

    nPlaneVert = len(plane_top.data.vertices)
    nMvert = 3*nPlaneVert

    #-- construct hexahedra
    if meshType==4:
      M = bpy.data.meshes.new(name = "tet_mesh")
    elif meshType==8:
      M = bpy.data.meshes.new(name = "hex_mesh")
    else:
      print("unexpected meshType indicator");return
    M.vertices.add(nMvert)
    for i in range(0,nPlaneVert):
        M.vertices[i].co = plane_mid.data.vertices[i].co
        M.vertices[nPlaneVert + i].co = plane_bot.data.vertices[i].co
        M.vertices[2*nPlaneVert + i].co = plane_top.data.vertices[i].co

    botVertices = [i + nPlaneVert for i in range(nPlaneVert)]
    topVertices = [i + 2*nPlaneVert for i in range(nPlaneVert)]
    nquad = len(plane_mid.data.polygons)
    for i in range(nquad):
        mid_quad = plane_mid.data.polygons[i]
        bot_quad = plane_bot.data.polygons[i]
        top_quad = plane_top.data.polygons[i]
        for j in range(4):
            bot_quad.vertices[j] = bot_quad.vertices[j] + nPlaneVert
            top_quad.vertices[j] = top_quad.vertices[j] + 2*nPlaneVert
        if meshType==4:
          createTets(M, (bot_quad, mid_quad),i)
          createTets(M, (mid_quad, top_quad),i+1)
        elif meshType==8:
          # determine if orientation by 4 vertices of one quad agrees with the direction pointing into the hex
          if i == 0:
            # direction of the 0123 orientation
            vec1 = plane_mid.data.vertices[mid_quad.vertices[1]].co
            vec0 = plane_mid.data.vertices[mid_quad.vertices[0]].co
            vec3 = plane_mid.data.vertices[mid_quad.vertices[3]].co
            vec01 = Vector((vec1[0]-vec0[0],vec1[1]-vec0[1],vec1[2]-vec0[2]))
            vec03 = Vector((vec3[0]-vec0[0],vec3[1]-vec0[1],vec3[2]-vec0[2]))
            dir0123 =  crossProd(vec01,vec03)
            # bottom to top direction of a hex
            midv0 = plane_mid.data.vertices[mid_quad.vertices[0]].co
            topv0 = plane_top.data.vertices[mid_quad.vertices[0]].co # index of mid_quad since this is the index for plane, not for mesh M
            bot2top = Vector((topv0[0]-midv0[0],topv0[1]-midv0[1],topv0[2]-midv0[2]))
            dotp = dotProd(dir0123,bot2top)
            if dotp>0:
              coDirection  = True
            elif dotp<0:
              coDirection = False
            else:
              print("three planes are too close"); assert(False)
          hex = M.hexahedra.add()
          for j in range(4):
              # bot-mid hex
              if coDirection:
                hex.vertices[j] = bot_quad.vertices[j]
                hex.vertices[4+j] = mid_quad.vertices[j]
              else:
                hex.vertices[j] = bot_quad.vertices[3-j]
                hex.vertices[4+j] = mid_quad.vertices[3-j]
          hex = M.hexahedra.add()
          for j in range(4):
              # mid-top hex
              if coDirection:
                hex.vertices[j] = mid_quad.vertices[j]
                hex.vertices[4+j] = top_quad.vertices[j]
              else:
                hex.vertices[j] = mid_quad.vertices[3-j]
                hex.vertices[4+j] = top_quad.vertices[3-j]

    recalc_outer_surface(M)
    ct.data = M

    ct['annotated_type'] = 'CONNECTIVETISSUE'
    ct['topObject'] = o1.name
    ct['botObject'] = o2.name
    ct['topVertices'] = topVertices
    ct['botVertices'] = botVertices

    ct['carvable'] = 1
    ct['collisionGroup'] = 1
    ct['contactFriction'] = 0.010
    ct['contactStiffness'] = 500
    ct['damping'] = 0.100
    ct['poissonRatio'] = 0.450
    ct['precomputeConstraints'] = 0
    ct['selfCollision'] = 0
    ct['suture'] = 0
    ct['youngModulus'] = 300
    ct['color'] = "white"

    bpy.ops.object.select_all(action='DESELECT')
    plane_top.select = True; bpy.ops.object.delete()
    plane_mid.select = True; bpy.ops.object.delete()
    plane_bot.select = True; bpy.ops.object.delete()

def createTets(m, quad_tuple, iteration):
    def isOdd(x): return (x % 2 != 0)

    q_idx = isOdd(iteration)

    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[1]
    tet.vertices[2] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[1]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]

    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[1] = quad_tuple[int(not q_idx)].vertices[2]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[1]
    tet.vertices[3] = quad_tuple[int(q_idx)].vertices[2]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]

    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[3]
    tet.vertices[2] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[3]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]

    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(not q_idx)].vertices[1]
    tet.vertices[1] = quad_tuple[int(not q_idx)].vertices[0]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[3] = quad_tuple[int(q_idx)].vertices[0]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[2] = tet.vertices[2], tet.vertices[0]

    tet = m.tetrahedra.add()
    tet.vertices[0] = quad_tuple[int(q_idx)].vertices[0]
    tet.vertices[1] = quad_tuple[int(q_idx)].vertices[2]
    tet.vertices[2] = quad_tuple[int(not q_idx)].vertices[3]
    tet.vertices[3] = quad_tuple[int(not q_idx)].vertices[1]
    if isOdd(iteration):
        tet.vertices[0], tet.vertices[1] = tet.vertices[1], tet.vertices[0]



def crossProd(v1,v2):
    return [v1[1]*v2[2] - v1[2]*v2[1],-(v1[0]*v2[2] - v1[2]*v2[0]),v1[0]*v2[1] - v1[1]*v2[0]]

def dotProd(v1,v2):
    return v1[0]*v2[0]+v1[1]*v2[1]+v1[2]*v2[2]

def register():
    bpy.utils.register_class(ConnectiveTissue)

def unregister():
    bpy.utils.unregister_class(ConnectiveTissue)
