### Scene 
The scene in Blender is the root context in SOFA. 

Gravity of the scene (defined in scene tab) is exported as global gravity in SOFA). 

Other custom properties supported:

|Cutom Property| Purpose |
|--------------| ------------------------------------------|
| includes | semi-colon seperated list of XML files to include in the resulting XML |
| displayFlags | space separated list of different details and debug drawing to show, refer to displayFlags in SOFA for syntax |

### Soft body objects
In order to model a soft body, start with a mesh or NURBS surface object. 

Set the custom property *annotated_type* to *SOFT_BODY*. 

Other custom properties supported:

|Cutom Property| Purpose |
|--------------| ------------------------------------------|
| youngModulus |  The stiffness of the simulated soft body |
| poissonRaio  |  Compressability of the simulated soft body |

In case of NURBS surface, it is evaluated before exporting it to XML. The *Preview U* and *Preview V* resolutions 
in object data tab are used to evaluate for collision model. The visual model is evaluated with *Render U* and *Render V* 
resolution in the same tab.

### Decorative visual models
For any model in the scene that doesn't have the property *annotated_type*, it will be exported as visual model only.
It supports texture mapping.

### Haptic tools
There are two ways to model haptic tools:
1. Set the custom property *annotated_type* of a model to *HAPTIC*. (For tools that only have one part)
2. Create an empty object, and set its custom property *annotated_tyoe* to *HAPTIC*.(For tool that has more than one part)

Other custom properties supported:

|Cutom Property| Purpose |
|--------------| ------------------------------------------|
| index |  Identify different parts for the haptic tools. Default value is 1 (For parts that are fixed) |

### Lights
There are two kinds of lights that are supported in the exportation:
1. SPOT LIGHT
2. POINT LIGHT 

### Obstacles
In order to model a collision model, start with a mesh or NURBS surface object. 

Set the custom property *annotated_type* to *COLLISION*. 

### Obstacles with different collision model
Steps to create a different collision model for an object.
1. Add an EMPTY object to the scene.
2. Create the visual model and collision model as the children of this empty object.
3. Set this empty object's custom property *annotated_type* to *CM*.
4. Set the collision model's custom property *annotated_type* to *COLLISIONMODEL*.
