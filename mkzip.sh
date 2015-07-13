zip -r blender2sofa-$(hg id -i).zip blender2sofa.py \
  export2sofa/__init__.py\
	export2sofa/ui.py\
  export2sofa/lua_export.py

zip -r blender-tetrahedral-$(hg id -i).zip blendertetrahedral.py \
  tetrahedral/__init__.py \
	tetrahedral/cgaltetrahedralize/__init__.py \
	tetrahedral/cgaltetrahedralize/libmpfr-4.dll\
	tetrahedral/cgaltetrahedralize/CGAL-vc120-mt-4.6.dll \
	tetrahedral/cgaltetrahedralize/CGALTetrahedralize.dll \
	tetrahedral/cgaltetrahedralize/msvcp120.dll\
	tetrahedral/cgaltetrahedralize/msvcr120.dll\
	tetrahedral/cgaltetrahedralize/libgmp-10.dll 
