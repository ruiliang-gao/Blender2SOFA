# Changelog 2.80 - 2.90 (Updated 3/7/2021)

## Version

In **init**.py:

- Line 5: Changed to `'blender': (2, 90, 0)`
- Changed deprecated `wiki_url` to `doc_url`

## Error Fixes

In export.py:

- Fixed light color exporting
- Update 'points' parameter name to 'position' for HexahedronSetTopologyContainer
- Remove deprecated object HexahedronSetTopologyAlgorithms
- Do not apply unrelated attribute 'damping' to HexahedronFEMForceField
- Fixed material/color exporting for objects
- Update data templates for several nodes
- Change several aliasas to the actual classes prevent issues when they are removed in the future
- Update addConstraints matrix-vector multiplication format
- Remove unused attribute "removeIsolated" for TriangleSetTopologyModifier
- Improved vector_to_string, export\*, and stringify_tree functions to avoid/handle exceptions better
- Improve memory safety when handling vectors
- Remove unecessary author-\* tags from AttachConstraints
- Prevent duplicate RequirePlugin nodes
- Change exportVisual to handle tris/quads better and prevent blender crashes
- Always add tearing threshold to ConnectingTissue to prevent SOFA issues
- Double check that with_transform is True while using exportVisual
- Correct object1/object2 to input/output for certain uses of BarycentricMapping
- Use to_mesh_clear after using to_mesh to prevent blender crashes
  In runsofa.py:
- Add opt.pluginAddedCT boolean to track whether the connecting tissue plugin has been added
  In thick_curve.py:
- Initialize bevel_resolution to 0
  In io_msh.py:
- Fix crash from using icon that no longer exists
  In ui.py:
- Remove out unused operator `option.convert_hex_to_tet`
- Fix .label() to use `text=` for hyperelastic
  In fattytissue.py:
- Update execute matrix-vector multiplication format
- Assume the Empty Cube is part of a collection rather than free in the scene
- Lower attachThreshold to prevent SOFA crash when points are projected
  In conn_tissue.py:
- Update to use new matrix-vector multiplication format
- Remove invalid line `ct.suture = True`
