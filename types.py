#!/usr/bin/env python
# -*- coding: utf-8 -*-
import bpy

def register_sofa_properties():
    #"""SOFA properties associated with a scene"""
    bpy.types.Scene.mu = bpy.props.FloatProperty(name=u"\u03bc",description="LCP parameter mu",soft_min=1e-9,soft_max=0.1,step=1e-6,default=1e-6,precision=6)
    bpy.types.Scene.alarmDistance = bpy.props.FloatProperty(name="Alarm Distance",description="Collision detection check distance",default=0.1,soft_min=1e-4,soft_max=0.1,step=1e-5,precision=3)
    bpy.types.Scene.contactDistance = bpy.props.FloatProperty(name="Contact Distance",default=0.01,soft_min=1e-5,soft_max=0.1,step=1e-5,precision=6)
    bpy.types.Scene.showXYZFrame = bpy.props.BoolProperty(name="Show XYZ frame",description="Show a small XYZ frame in the lower right corner in SOFA simulation",default=False)
    bpy.types.Scene.hapticWorkspaceBox = bpy.props.StringProperty(name="Haptic Workspace Box",description="An empty object that defines the haptic workspace box")

    #"""SOFA properties and annotations for objects"""
    bpy.types.Object.template = bpy.props.EnumProperty(name="Template",default='VISUAL', items=[
        ('VISUAL', 'Visual', 'A decorative visual object that does not participate in simulation', 'SCENE', 1),
        ('VOLUMETRIC', 'Volumetric', 'A hexahedral or tetrahedral volumetric mesh', 'SNAP_VOLUME', 2),
        ('THICKSHELL', 'Thick Shell', 'An offset object as a thick shell', 'MOD_CLOTH', 3),
        ('THICKCURVE', 'Thick Curve', 'A thick curve made from a Bezier object', 'ROOTCURVE', 4),
        ('CLOTH', 'Cloth', 'A surface cloth', 'OUTLINER_OB_SURFACE', 5),
        ('COLLISION', 'Obstacle', '', 'SOLID', 6),
        ('SPHERECONSTRAINT','Sphere Constraint','', 'SURFACE_NSPHERE', 7),
        ('BOXCONSTRAINT', 'Box Constraint', '','OBJECT_DATA', 8),
        ('ATTACHCONSTRAINT', 'Spring Attachment', '', 'LINKED', 9),
        ('INSTRUMENT','Haptic Instrument','A haptically enabled surgical instrument', 'SCULPTMODE_HLT',10),
        ('INSTRUMENTPART', 'Intrument part', 'An animated part of the instrument', 'OOPS', 11),
        ('INSTRUMENTTIP','Tip of Instrument','Active part of the instrument that performs actions', 'OOPS', 12),
        ('INSTRUMENTCOLLISION','Collision part of Instrument','Collision part of the instrument along the shaft', 'OOPS', 13)
        # Rigid does not work as expected. It is hidden until it is fixed
        # ('RIGID', 'Rigid', '', 'SOLID', 13),
        ])

    # Instrument properties
    bpy.types.Object.toolFunction = bpy.props.EnumProperty(name="Function",description="Interactive function of an instrument", default='GRASP',items=[
        ('GRASP', 'Grasp', 'A grasper instrument'),
        ('SUTURE','Suture', 'A grasper that can be used for suturing'),
        ('CARVE', 'Carve', 'An instrument that destroys tissue at contact'),
        ('CLAMP', 'Clamp', 'Apply clips to vessels to close them')
        ])
    bpy.types.Object.instrumentPart = bpy.props.EnumProperty(name="Animated Part type",default='FIXED',items=[
        ('LEFTJAW', 'Left Jaw', ''),
        ('RIGHTJAW', 'Right Jaw', ''),
        ('FIXED', 'Fixed', 'Fixed part of the tool that moves with the handle')
        ])
    bpy.types.Object.proximity = bpy.props.FloatProperty(name="Proximity",description="Proximity for collision detection",min=0,default=0,max=10,step=0.01)

    # Collision detection and response
    bpy.types.Object.collisionGroup = bpy.props.IntProperty(name="Collision Group",default=1,min=1,max=100,soft_max=10)
    bpy.types.Object.selfCollision = bpy.props.BoolProperty(name="Self Collision",description="Object cannot go through itself when enabled",default=False)
    bpy.types.Object.contactFriction = bpy.props.FloatProperty(name="Contact Friction",default=500,min=0,max=1e+5,step=100)
    bpy.types.Object.contactStiffness = bpy.props.FloatProperty(name="Contact Stiffness",default=500,min=0,max=1e+5,step=100)

    # Elasticity
    bpy.types.Object.youngModulus = bpy.props.FloatProperty(name="Stiffness (Young Modulus)",default=3000,min=1,max=1e+6,soft_min=10,step=100)
    bpy.types.Object.poissonRatio = bpy.props.FloatProperty(name="Compressibility",default=0.45,min=0.0,max=0.49,step=0.01)
    bpy.types.Object.rayleighStiffness = bpy.props.FloatProperty(name="Rayleigh Stiffness",default=0.0,min=0.0,max=0.49,step=0.01)
    bpy.types.Object.bendingStiffness = bpy.props.FloatProperty(name="Bending Stiffness",default=3000,min=1,max=1e+6,step=100)
    bpy.types.Object.damping = bpy.props.FloatProperty(name="Damping",default=0.1,min=0,max=1000,step=0.1)
    bpy.types.Object.precomputeConstraints = bpy.props.BoolProperty(name='Accurate Constraints',description='Better and more accurate constraints but requires lengthy precomputation',default=False)

    # Attachments
    bpy.types.Object.attachStiffness = bpy.props.FloatProperty(name="Attach Stiffness",default=10000,min=1,max=1e+6,soft_min=10,step=100)
    bpy.types.Object.alwaysMatchForObject1 = bpy.props.BoolProperty(name='Always Match for First Object',default=False)
    bpy.types.Object.alwaysMatchForObject2 = bpy.props.BoolProperty(name='Always Match for Second Object',default=False)
    bpy.types.Object.object1 = bpy.props.StringProperty(name='First Object', description='Name of the first object in the attachment')
    bpy.types.Object.object2 = bpy.props.StringProperty(name='Second Object', description='Name of the second object in the attachment')
    bpy.types.Object.attachThreshold = bpy.props.FloatProperty(name="Attach Threshold",default=0.02,min=0.001,max=1.0,step=0.01,description='Maximum distance between connected vertices of two objects as a percentage of the size of object')
    
    # Interactive features
    bpy.types.Object.carvable = bpy.props.BoolProperty(name='Carvable',description='Allow the object be interactively carved by mouse or a carving tool',default=False)
    bpy.types.Object.suture = bpy.props.BoolProperty(name='Interactive',description='Allow the object to be interactively manipulated by the haptic tools',default=True)

    #
    bpy.types.Object.texture3d = bpy.props.StringProperty(name='3D Texture',description='Filename of the 3D texture')
    bpy.types.Object.thickness = bpy.props.FloatProperty(name='Thickness',description='Thickness of the shell', default=0.1,min=0.001,max=1,step=0.01)
    bpy.types.Object.layerCount = bpy.props.IntProperty(name='Layer Count', description='Number of layers in the thick shell',default=1,min=1,max=10)

def unregister_sofa_properties():
    #"""SOFA properties associated with a scene"""
    del bpy.types.Scene.mu
    del bpy.types.Scene.alarmDistance
    del bpy.types.Scene.contactDistance
    del bpy.types.Scene.showXYZFrame
    del bpy.types.Scene.hapticWorkspaceBox

    #"""SOFA properties and annotations for objects"""
    del bpy.types.Object.template
    # Instrument properties
    del bpy.types.Object.toolFunction
    del bpy.types.Object.instrumentPart
    del bpy.types.Object.proximity
    # Collision detection and response
    del bpy.types.Object.collisionGroup
    del bpy.types.Object.selfCollision
    del bpy.types.Object.contactFriction
    del bpy.types.Object.contactStiffness

    # Elasticity
    del bpy.types.Object.youngModulus
    del bpy.types.Object.poissonRatio
    del bpy.types.Object.rayleighStiffness
    del bpy.types.Object.bendingStiffness
    del bpy.types.Object.damping
    del bpy.types.Object.precomputeConstraints

    # Attachments
    del bpy.types.Object.attachStiffness
    del bpy.types.Object.alwaysMatchForObject1
    del bpy.types.Object.alwaysMatchForObject2
    del bpy.types.Object.object1
    del bpy.types.Object.object2

    # Interactive features
    del bpy.types.Object.carvable
    del bpy.types.Object.suture

    #
    del bpy.types.Object.texture3d
    del bpy.types.Object.thickness
    del bpy.types.Object.layerCount

class HapticProperties(bpy.types.PropertyGroup):
    scale = bpy.props.FloatProperty(name='Workspace Scale',description='Scaling applied to the workspace box of the haptic',default=300,min=1,max=10000,step=10)
    forceScale = bpy.props.FloatProperty(name='Force-feedback Scale',description='Scaling applied to force feedback',default=0.03,min=0,max=10000,soft_max=1)
    forceFeedback = bpy.props.BoolProperty(name='Force-feedback enabled',description='Enable force-feedback for this haptic device',default=False)
    deviceName = bpy.props.StringProperty(name='Device Name',description='Name of the haptic device name as registered in the Geomagic Touch Setup application')

