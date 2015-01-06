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


### Haptic tools

### Lights

### Obstacles

### Obstacles with different collision model
