# gaitmesh

This package processes a mesh model (or point cloud and mesh) to produce an annotated version of the mesh where each triangle is associated with a choice of gait-controller for a legged robot.
The result can then be used in the package recast_ros for multi-controller long-term locomotion planning for a legged robot.

If you use this in your research, please cite:

> Martim Brandao, Omer Burak Aladag, and Ioannis Havoutis, "**GaitMesh: controller-aware navigation meshes for long-range legged
locomotion planning in multi-layered environments**", in *ICRA2020* (submitted).

## From a CAD model

As an example, please download the [Barcelona Robotics Lab model](https://www.iri.upc.edu/research/webprojects/pau/datasets/BRL/zip/BRL_obj_wireframe.zip), and then run:

```
roslaunch gaitmesh_ros annotate_cloud_gaits.launch input_clouds_not_mesh:=false
```

## From a point cloud

If you have your own point cloud, obtained for example by laser scanning of a site, then follow these steps to produce a triangular mesh:

- Load the cloud in MeshLab
- In MeshLab, run "Surface Reconstruction: Ball Pivoting" with a reasonable radius, e.g. 0.20 meters.
- In MeshLab, run "Close holes" with a reasonable max size, e.g. 40.
- Change "annotate_cloud_gaits.launch" to include the path to your cloud in "cloud1" and the path to the mesh in "cloud_mesh".

Then run:

```
roslaunch gaitmesh_ros annotate_cloud_gaits.launch
```

