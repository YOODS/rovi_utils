import copy
import numpy as np
import open3d as o3


def show(model, scene, model_to_scene_trans=np.identity(4)):
    model_t = copy.deepcopy(model)
    scene_t = copy.deepcopy(scene)

    model_t.paint_uniform_color([1, 0, 0])
    scene_t.paint_uniform_color([0, 0, 1])

    model_t.transform(model_to_scene_trans)

    o3.visualization.draw_geometries([model_t, scene_t])
