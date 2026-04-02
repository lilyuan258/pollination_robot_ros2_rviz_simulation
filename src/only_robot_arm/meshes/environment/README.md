# Environment Meshes

Put your SolidWorks-exported environment meshes in this folder, for example:

- `rail.dae` or `rail.stl`
- `column.dae` or `column.stl`
- `platform.dae` or `platform.stl`

Then edit:

- `only_robot_arm/config/environment_visual.yaml`

and set `rail_mesh_resource`, `column_mesh_resource`, `platform_mesh_resource` to the corresponding `package://` URI.

Example:

`package://only_robot_arm/meshes/environment/rail.dae`

If you already have a standalone `environment` package with a single mesh
(`package://environment/meshes/environment_link.STL`), you can directly use it in
`environment_visual.yaml` and disable `column/platform`.
