#!/usr/bin/env python
# -*- coding: utf-8 -*-
import bpy

def register_sofa_properties():
    #"""SOFA properties associated with a scene"""
    bpy.types.Scene.mu = bpy.props.FloatProperty(name=u"\u03bc",description="LCP parameter mu",soft_min=1e-9,soft_max=0.1,step=1e-6,default=1e-6,precision=6)
    bpy.types.Scene.alarmDistance = bpy.props.FloatProperty(name="Alarm Distance",description="Collision detection check distance",default=0.1,soft_min=1e-4,soft_max=0.1,step=1e-5,precision=3)
    bpy.types.Scene.contactDistance = bpy.props.FloatProperty(name="Contact Distance",default=0.01,soft_min=1e-5,soft_max=0.1,step=1e-5,precision=6)
    bpy.types.Scene.showXYZFrame = bpy.props.BoolProperty(name="Show XYZ frame",description="Show a small XYZ frame in the lower right corner in SOFA simulation",default=False)
    bpy.types.Scene.precompution = bpy.props.BoolProperty(name="Precompution",description="Check if there are any objects to be precomputed in the scene",default=False)
    bpy.types.Scene.haptic1WorkspaceBox = bpy.props.StringProperty(name="Haptic1 Workspace Box",description="An empty object that defines the haptic 1 workspace box")
    bpy.types.Scene.haptic2WorkspaceBox = bpy.props.StringProperty(name="Haptic2 Workspace Box",description="An empty object that defines the haptic 2 workspace box")
    bpy.types.Scene.hapticMoveTo = bpy.props.StringProperty(name="Haptic Move To Position",description="An object that defines where the haptic moves to when simulation starts")
    bpy.types.Scene.alignOmniWithCamera = bpy.props.BoolProperty(name="Align Omni With Camera",description="align Omni position and orientation with camera",default=False)
    bpy.types.Scene.defaultInstrument = bpy.props.StringProperty(name="Default instrument",description="A tool object that defines the default instrument")
    bpy.types.Scene.useSpeechRecognition = bpy.props.BoolProperty(name="SpeechRecognition", description="check this if you want to use SpeechRecognition plugin", default=False)
    bpy.types.Scene.targetOrgan = bpy.props.StringProperty(name="Target Organ",description="The target organ for this procedure, will be used for triggering the completion")
    bpy.types.Scene.sharePath = bpy.props.StringProperty(name="SOFA mesh filepath",description="Specify SOFA's mesh/TIPS filepath here")
    bpy.types.Scene.versionSOFA = bpy.props.StringProperty(name="SOFA version number",default="18",description="Specify SOFA's version number here, type ‘18’ for SOFA1812 or later")
    bpy.types.Scene.enableEndoscope = bpy.props.BoolProperty(name="Use EndoscopeController",description="Check if you have included endoscope tool in the scene",default=False)
    bpy.types.Scene.enableSutureController = bpy.props.BoolProperty(name="Use SutureController",description="Check if you have included the python SutureController in the scene",default=False)
    bpy.types.Scene.sutureOrgan1 = bpy.props.StringProperty(name="Suture Organ 1",description="The first organ to be sutured for this procedure")
    bpy.types.Scene.sutureOrgan2 = bpy.props.StringProperty(name="Suture Organ 2",description="The second organ to be sutured for this procedure")
    #"""SOFA properties and annotations for objects"""
    bpy.types.Object.template = bpy.props.EnumProperty(name="Template",default='VISUAL', items=[
        ('VISUAL', 'Visual', 'A decorative visual object that does not participate in simulation', 'SCENE', 1),
        ('VOLUMETRIC', 'Volumetric', 'A hexahedral or tetrahedral volumetric mesh', 'SNAP_VOLUME', 2),
        ('THICKSHELL', 'Thick Shell', 'An offset object as a thick shell', 'MOD_CLOTH', 3),
        ('THICKCURVE', 'Thick Curve', 'A thick curve made from a Bezier object', 'ROOTCURVE', 4),
        ('CLOTH', 'Cloth', 'A surface cloth', 'OUTLINER_OB_SURFACE', 5),
        ('COLLISION', 'Obstacle', '', 'SOLID', 6),
        ('SPHERECONSTRAINT','Sphere Constraint','To constrain a certain object, put this sphere object as a child object and all vertices inside this sphere will be fixed ', 'SURFACE_NSPHERE', 7),
        ('BOXCONSTRAINT', 'Box Constraint', '','OBJECT_DATA', 8),
        ('ATTACHCONSTRAINT', 'Spring Attachment', 'specify the two objects that you want to attach, this sphere object should cover the area where springs are generated', 'LINKED', 9),
        ('INSTRUMENT','Haptic Instrument','A haptically enabled surgical instrument', 'SCULPTMODE_HLT',10),
        ('INSTRUMENTPART', 'Intrument part', 'An animated part of the instrument', 'OOPS', 11),
        ('INSTRUMENTTIP','Tip of Instrument','Active part of the instrument that performs actions', 'OOPS', 12),
        ('INSTRUMENTCOLLISION','Collision part of Instrument' ,'Collision part of the instrument along the shaft', 'OOPS', 13),
        ('SAFETYSURFACE', 'Safety surface','An surface object that used for safety detection', 'MOD_SUBSURF', 14),
        ('DEFORMABLE', 'Defomable Grid ','An deformable surface object that is embeded in a grid structure: To use it, first export .obj to SOFA/share/mesh/TIPS. Also make sure there is no spaces in object name as SOFA visual loader can not parse that', 'LATTICE_DATA', 15),
        # Rigid does not work as expected. It is hidden until it is fixed
        ('RIGID', 'Rigid', '', 'SOLID', 16)
        ])

    # Instrument properties
    bpy.types.Object.toolFunction = bpy.props.EnumProperty(name="Function",description="Interactive function of an instrument", default='GRASP',items=[
        ('GRASP', 'Grasp', 'A grasper instrument'),
        ('SUTURE','Suture', 'A grasper that can be used for suturing'),
        ('CARVE', 'Cauterize', 'An instrument that destroys tissue at contact, may destroy veins'),
        ('DISSECT','Dissect', 'An instrument that dissects both tissue and veins at contact'),
        ('CUT','Cut', 'An instrument that dissects tissue but does nothing on the veins'),
        ('CLAMP', 'Clamp', 'Apply clips to vessels to close them'),
        ('CONTAIN', 'Contain', 'Container of the orgrans'),
        ('CAMERA', 'Camera', 'Endoscope tool')
        ])
    bpy.types.Object.instrumentPart = bpy.props.EnumProperty(name="Animated Part type",default='FIXED',items=[
        ('LEFTJAW', 'Left Jaw', 'left jaw of an instrument that rotate around Y-axis'),
        ('RIGHTJAW', 'Right Jaw', 'right jaw of an instrument that rotate around X-axis'),
        ('FIXED', 'Fixed', 'Fixed part of the tool that moves with the handle'),
        ('LEFTCLIP', 'Left clip', 'left jaw of clip applier that rotate around X-axis'),
        ('RIGHTCLIP', 'Right clip', 'right jaw of clip applier that rotate around X-axis'),
        ('TOOLSHAFT', 'Tool shaft', 'shaft of the tool'),
        ('TOOLVISUALEFFECT', 'Tool visual effect', 'the part that renders the visual effects')
        ])
    bpy.types.Object.proximity = bpy.props.FloatProperty(name="Proximity",description="Proximity: enlargement of its collision model",min=0,default=0,max=10,step=0.001)
    bpy.types.Object.extraTag = bpy.props.StringProperty(name="Extra Tag",description='Put extra tag to the object that SOFA can access it')

    # Collision detection and response
    bpy.types.Object.collisionGroup = bpy.props.IntProperty(name="Collision Group",default=1,min=1,max=100,soft_max=10)
    bpy.types.Object.selfCollision = bpy.props.BoolProperty(name="Self Collision",description="Object cannot go through itself when enabled",default=False)
    bpy.types.Object.contactFriction = bpy.props.FloatProperty(name="Contact Friction",default=500,min=0,max=1e+5,step=100)
    bpy.types.Object.contactStiffness = bpy.props.FloatProperty(name="Contact Stiffness",default=500,min=0,max=1e+5,step=100)
    bpy.types.Object.alternativeCollision = bpy.props.StringProperty(name='Collision Model', description='Name of the object that will be used as collision model. Equal physical model if left blank')
    
    # Elasticity
    bpy.types.Object.youngModulus = bpy.props.FloatProperty(name="Stiffness (Young Modulus)",description=' Modulus of elasticity which represents how easy it is to deform (stretch a material)',default=3000,min=1,max=1e+6,soft_min=10,step=100)
    bpy.types.Object.poissonRatio = bpy.props.FloatProperty(name="Compressibility (PoissonRatio)",description = 'Measures how much will the material expand in directions perpendicular to the direction of compression',default=0.45,min=0.0,max=0.49,step=0.01)
    bpy.types.Object.rayleighStiffness = bpy.props.FloatProperty(name="Rayleigh Stiffness",description = 'Rayleigh damping is viscous damping that is proportional to a linear combination of mass and stiffness',default=0.0,min=0.0,max=0.49,step=0.01)
    bpy.types.Object.bendingStiffness = bpy.props.FloatProperty(name="Bending Stiffness", description = 'the resistance of a member against bending deformation.', default=3000,min=1,max=1e+6,step=100)
    bpy.types.Object.damping = bpy.props.FloatProperty(name="Damping",default=0.1,min=0,max=1000,step=0.1)
    bpy.types.Object.precomputeConstraints = bpy.props.BoolProperty(name='Accurate Constraints',description='(Currently unstable)Better and more accurate constraints but requires lengthy precomputation',default=False)
    bpy.types.Object.totalMass = bpy.props.FloatProperty(name="Mass Density",default=0.05,min=0.001,max=100,step=0.01)
    
    # Other Material Types & Params
    bpy.types.Object.materialType = bpy.props.EnumProperty(name="Material type",default='ELASTIC',items=[
        ('ELASTIC', 'Elasticity', 'use default elastic material with CorotationalFEM'),
        ('PLASTIC', 'Plasticity', 'use plastic material'),
        ('HYPERELASTIC', 'Hyperelasticity', 'use hyperelastic material')   
        ])

    bpy.types.Object.materialName = bpy.props.EnumProperty(name="Material name",default='StVenantKirchhoff',items=[
        ('StVenantKirchhoff', 'StVenantKirchhoff', 'StVenantKirchhoff material'),
        ('ArrudaBoyce', 'ArrudaBoyce', 'ArrudaBoyce material'),
        ('NeoHookean', 'NeoHookean', 'NeoHookean material'),
        ('MooneyRivlin', 'MooneyRivlin', 'MooneyRivlin material')
        ])
    bpy.types.Object.plasticYieldThreshold = bpy.props.FloatProperty(name="plasticYieldThreshold",description='plasticYieldThreshold',default=0.005,min=0,max=1,soft_min=0,step=0.001)
    bpy.types.Object.plasticMaxThreshold = bpy.props.FloatProperty(name="plasticMaxThreshold",description='plasticMaxThreshold',default=0.5,min=0,max=1,soft_min=0,step=0.01)
    bpy.types.Object.plasticCreep = bpy.props.FloatProperty(name="plasticCreep",description='plasticCreep',default=0.1,min=0,max=1,soft_min=0,step=0.01)
    
    # Attachments
    bpy.types.Object.attachStiffness = bpy.props.FloatProperty(name="Attach Stiffness",default=10000,min=1,max=1e+6,soft_min=10,step=100)
    bpy.types.Object.naturalLength = bpy.props.FloatProperty(name="Spring Natural Length",default=0.5,min=0.1,max=10,step=0.1)
    bpy.types.Object.alwaysMatchForObject1 = bpy.props.BoolProperty(name='Always Match for First Object',default=False)
    bpy.types.Object.alwaysMatchForObject2 = bpy.props.BoolProperty(name='Always Match for Second Object',default=False)
    bpy.types.Object.useBilateralConstraint = bpy.props.BoolProperty(name='use bilateralconstraint',description='if true, use bilateralconstraint instead of springs system',default=False)
    bpy.types.Object.object1 = bpy.props.StringProperty(name='First Object', description='Name of the first object in the attachment')
    bpy.types.Object.object2 = bpy.props.StringProperty(name='Second Object', description='Name of the second object in the attachment')
    bpy.types.Object.attachThreshold = bpy.props.FloatProperty(name="Attach Threshold",default=0.02,min=0.001,max=1.0,step=0.001,precision=3,description='Maximum distance between connected vertices of two objects as a percentage of the size of object')
    bpy.types.Object.tearingThreshold = bpy.props.FloatProperty(name="Spring Tearing Threshold",default=3.0,min=1.0,max=100.0,step=0.1,precision=2,description='threshold of the deform ratio(deformedLength / restLength) for tearing the spring')

    # Interactive features
    bpy.types.Object.carvable = bpy.props.BoolProperty(name='Carvable',description='Allow the object be interactively carved by mouse or a carving tool',default=False)
    bpy.types.Object.interactive = bpy.props.BoolProperty(name='Interactive',description='Allow the object to be interactively manipulated by the haptic tools',default=True)
    
    #Some Constraints
    bpy.types.Object.fixed_indices = bpy.props.StringProperty(name='Fixed_Indices',description='Vertex indices used for fixed constraints')
    bpy.types.Object.fixed_direction = bpy.props.StringProperty(name='Fixed Directions',description='Directions for fixed constraints, e.g 0 0 1 to fix along Z-axis')
    bpy.types.Object.local_gravity = bpy.props.StringProperty(name='Local Gravity',description='A Local force vector that will be applied on the object, like gravity. Input three numbers for XYZ, like "0 0 -9.8"')
    
	#Rendering
    bpy.types.Object.texture3d = bpy.props.StringProperty(name='3D Texture',description='Filepath of the 3D texture, relative path of share/textures/')
    bpy.types.Object.texture2d = bpy.props.StringProperty(name='2D Texture',description='Filepath of the 2D texture, relative path of share/textures/')
    bpy.types.Object.useShader = bpy.props.BoolProperty(name='UseShader',description='Use our default shader for rendering this object',default=False)
    bpy.types.Object.shaderFile = bpy.props.StringProperty(name='shader location',description='Shader file location')
    bpy.types.Object.useTessellation = bpy.props.BoolProperty(name='useTessellation',description='Use the Tessellation shader for rendering this object',default=False)
    
    #Geometry
    bpy.types.Object.grid_dimension = bpy.props.StringProperty(name='Grid Dimensions',description='Specify the dimensions of the grid stucture that embeds the obj. Input three numbers for XYZ, like "4 3 2"')
    bpy.types.Object.thickness = bpy.props.FloatProperty(name='Thickness',description='Thickness of the shell', default=0.1,min=0.001,max=1,step=0.01)
    bpy.types.Object.layerCount = bpy.props.IntProperty(name='Layer Count', description='Number of layers in the thick shell',default=1,min=1,max=10)
    
    #Safety
    bpy.types.Object.safetyForceThreshold = bpy.props.FloatProperty(name='Safety Force Threshold', description='Maximum force a vein can withstand without sustaining injury', default=4.0)
    bpy.types.Object.safetyConcern = bpy.props.BoolProperty(name='Safety Organ', description='is a safety organ?',default=False)
    
def unregister_sofa_properties():
    #"""SOFA properties associated with a scene"""
    del bpy.types.Scene.mu
    del bpy.types.Scene.alarmDistance
    del bpy.types.Scene.contactDistance
    del bpy.types.Scene.showXYZFrame
    del bpy.types.Scene.precompution
    del bpy.types.Scene.haptic1WorkspaceBox
    del bpy.types.Scene.haptic2WorkspaceBox
    del bpy.types.Scene.hapticMoveTo
    del bpy.types.Scene.alignOmniWithCamera   
    
    del bpy.types.Scene.defaultInstrument
    del bpy.types.Scene.useSpeechRecognition
    del bpy.types.Scene.targetOrgan
    del bpy.types.Scene.sharePath
    del bpy.types.Scene.versionSOFA

    #"""SOFA properties and annotations for objects"""
    del bpy.types.Object.template
    # Instrument properties
    del bpy.types.Object.toolFunction
    del bpy.types.Object.instrumentPart
    del bpy.types.Object.proximity
    del bpy.types.Object.extraTag
    # Collision detection and response
    del bpy.types.Object.collisionGroup
    del bpy.types.Object.selfCollision
    del bpy.types.Object.contactFriction
    del bpy.types.Object.contactStiffness
    del bpy.types.Object.alternativeCollision

    # Elasticity
    del bpy.types.Object.youngModulus
    del bpy.types.Object.poissonRatio
    del bpy.types.Object.rayleighStiffness
    del bpy.types.Object.bendingStiffness
    del bpy.types.Object.damping
    del bpy.types.Object.precomputeConstraints
    del bpy.types.Object.totalMass

    # Attachments
    del bpy.types.Object.attachStiffness
    del bpy.types.Object.naturalLength
    del bpy.types.Object.alwaysMatchForObject1
    del bpy.types.Object.alwaysMatchForObject2
    del bpy.types.Object.object1
    del bpy.types.Object.object2

    # Interactive features
    del bpy.types.Object.carvable
    del bpy.types.Object.interactive
    del bpy.types.Object.fixed_indices
    del bpy.types.Object.fixed_direction
    del bpy.types.Object.local_gravity

    #Rendering and safety
    del bpy.types.Object.texture3d
    del bpy.types.Object.texture2d
    del bpy.types.Object.thickness
    del bpy.types.Object.layerCount
    del bpy.types.Object.useShader
    del bpy.types.Object.useTessellation
    del bpy.types.Object.shaderFile
    del bpy.types.Object.safetyForceThreshold
    del bpy.types.Object.safetyConcern
    
class HapticProperties(bpy.types.PropertyGroup):
    scale = bpy.props.FloatProperty(name='Workspace Scale',description='Scaling applied to the workspace box of the haptic',default=25,min=1,max=10000,step=10)
    forceScale = bpy.props.FloatProperty(name='Force-feedback Scale',description='Scaling applied to force feedback',default=0.0008,min=0,precision=5,max=10000,soft_max=1)
    forceFeedback = bpy.props.BoolProperty(name='Force-feedback enabled',description='Enable force-feedback for this haptic device',default=False)
    deviceName = bpy.props.StringProperty(name='Device Name',description='Name of the haptic device name as registered in the Geomagic Touch Setup application')
