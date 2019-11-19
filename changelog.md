# Changelog 2.79 - 2.80

## Version
In __init__.py:
'blender': (2, 80, 0)

## Registration
To register a class, add the class to the ```classes``` list in __init__.py, the class will be registered and unregistered in the for loops

```
for c in classes:
        bpy.utils.register_class(c)
```

Some classes still need to call a function to register more (register_other())

## Naming conventions
context.user_preferences is now just context.preferences

INFO_MT_ is now TOPBAR_MT_ (this was not documented in Blender's API changes)

* labels (and other functions) now require keyword arguments, for example:
```
layout.label('Haptic properties')
is now
layout.label(text='Haptic properties')
```

* class variables now must be annotations
```
cube = bpy.props.StringProperty(name = 'Sampling Cube', description = 'The cube used for sampling')
is now
cube: bpy.props.StringProperty(name = 'Sampling Cube', description = 'The cube used for sampling')
```

## Math
Matrix multiply is now @ instead of *, you can see this change on export.py near line 1316
```
lookAt = o.matrix_world @ Vector((0,0,-1))
```