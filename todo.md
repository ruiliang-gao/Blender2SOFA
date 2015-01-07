## SparseGrid support for CM objects
If an empty object with *CM* annotated type, has a child with annotated type *SPARSEGRID*, you should generate two tags
    
    <HexahedronFEMForceField ... />
    <SparseGridTopology />
    
The procedure for generating SparseGridTopology tag is very similar to MeshTopology (except for the change in tag name).

## Rigid vs. Deformable support for CM objects
If there is an custom property *template*, then use it to determine what tags to generate in the root level for CM object
If *template* is *Rigid* then following tags should be generated:
    
    <MechanicalObject template="Rigid" ... />
    
If *template* is *Deformable* then the following tags should be generated:
    <EulerImplicitSolver />
    <CGLinearSolver template="GraphScattered" />
    <MechanicalObject template="Vec3d" ... />
    <UniformMass />
    
There is also a change in how mappings are generated for Visual and Collision objects, what is currently implemented
only works for *Deformable* and is like the following for collision:
    <BarycentricMapping template="Vec3d,Vec3d" ... />
and following for visual:
    <BarycentricMapping template="Vec3d,ExtVec3f" ... />

    
For the case of *Rigid* instead of that we want for collision:
    <RigidMapping template="Rigid,Vec3d" ... />
and for visual:
    <RigidMapping template="Rigid,ExtVec3f" ... />

