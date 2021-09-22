import open3d as o3d
import sys 
import open3d.visualization.gui as gui
import numpy as np
import matplotlib.pyplot as plt

#show a cloud and rotate it
def rotate(cloud):
	def custom_draw_geometry_with_rotation(cloud):

		def rotate_view(vis):
			ctr = vis.get_view_control()
			ctr.rotate(10.0, 0.0)
			return False

		o3d.visualization.draw_geometries_with_animation_callback([cloud],rotate_view)

	custom_draw_geometry_with_rotation(cloud)

def no_rotation(cloud):
	
	gui.Application.instance.initialize()
	w = gui.Application.instance.create_window("Open3D Example - Events",
											   640, 480)
	scene = gui.SceneWidget()
	scene.scene = o3d.visualization.rendering.Open3DScene(w.renderer)
	w.add_child(scene)

	obj = o3d.geometry.TriangleMesh.create_moebius()

	material = o3d.visualization.rendering.Material()
	material.shader = "defaultLit"
	scene.scene.add_geometry("cloud", cloud, material)

	scene.setup_camera(60, scene.scene.bounding_box, (0, 0, 0))
	# scene.set_view_controls(gui.SceneWidget.Controls.FLY)

	gui.Application.instance.run()

def estimate_normals(pcd):
	#gt_mesh = o3dtut.get_bunny_mesh()
	#pcd = gt_mesh.sample_points_poisson_disk(5000)
	# o3d.geometry.estimate_normals(
	#         cloud,
	#         search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1,
	#                                                           max_nn=30))
	pcd.normals = o3d.utility.Vector3dVector(np.zeros(
	    (1, 3)))  # invalidate existing normals

	pcd.estimate_normals()
	#o3d.visualization.draw_geometries([pcd], point_show_normal=True)
	pcd.orient_normals_consistent_tangent_plane(100)
	#o3d.visualization.draw_geometries([pcd], point_show_normal=True)


	#pcd.estimate_normals()
	#o3d.visualization.draw_geometries([pcd], point_show_normal=True)
	return pcd

#point cloud to mesh
def reconstruction(cloud, output_path):
	poisson_mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(cloud, depth=9)
	
	
	#no_rotation(p_mesh_crop)

	print('visualize densities')
	densities = np.asarray(densities)
	density_colors = plt.get_cmap('plasma')(
		(densities - densities.min()) / (densities.max() - densities.min()))
	density_colors = density_colors[:, :3]
	density_mesh = o3d.geometry.TriangleMesh()
	density_mesh.vertices = poisson_mesh.vertices
	density_mesh.triangles = poisson_mesh.triangles
	density_mesh.triangle_normals = poisson_mesh.triangle_normals
	density_mesh.vertex_colors = o3d.utility.Vector3dVector(density_colors)
	no_rotation(density_mesh)
	print('remove low density vertices')
	vertices_to_remove = densities < np.quantile(densities, 0.09)#0.01, 0.02, 0.03, 0.04
	poisson_mesh.remove_vertices_by_mask(vertices_to_remove)
	#clean the mesh
	bbox = cloud.get_axis_aligned_bounding_box()
	p_mesh_crop = poisson_mesh.crop(bbox)
	#o3d.io.write_triangle_mesh(output_path+"flower_mesh_bin.obj", p_mesh_crop)#, write_ascii =True)
	print(poisson_mesh)
	no_rotation(poisson_mesh)
	return poisson_mesh

def main():
	o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
	output_path = "C:\\roomie3d\\models\\lounge\\"
	name = "result_61"
	#cloud = o3d.io.read_point_cloud("C:\\Users\\tairc\\Documents\\3d\\models\\520-sitescape_with_normals.ply")
	#cloud = o3d.io.read_point_cloud(output_path + "flowerPoints.ply")
	mesh = o3d.io.read_triangle_mesh(output_path + name +".obj")
	#no_rotation(mesh)
	#cloud = o3d.io.read_point_cloud(output_path + name +".ply")
	#cloud = o3d.io.read_point_cloud(output_path + name +"with_normals.ply")
	#o3d.io.write_point_cloud(output_path + "flowerPoints_bin.ply", cloud) #, write_ascii=True)
	
	
	
	# 
	# if len(cloud.normals) == 0:
	# 	print("adding normals")
	# 	cloud = estimate_normals(cloud)
	# 	o3d.io.write_point_cloud(output_path+name+"with_normals.ply", cloud)#, write_ascii=True)
	#cloud = estimate_normals(cloud)
	no_rotation(mesh)
	#return
	#print("Downsample the point cloud with a voxel of 0.05")
	#cloud = cloud.voxel_down_sample(0.05)
	#no_rotation(cloud)
	# print("Recompute the normal of the downsampled point cloud")
	#add normals
	#estimate_normals(cloud)
	#o3d.io.write_point_cloud(output_path+name+"with_normals.ply", cloud)#, write_ascii=True)
	#no_rotation(cloud)
	#paint uniform
	#chair.paint_uniform_color([1, 0.706, 0])

	#reconstruct mesh from point cloud
	mesh = reconstruction(cloud, output_path)
	o3d.io.write_triangle_mesh(output_path+name+".obj", mesh, write_ascii =True)


if __name__ == "__main__":
	main()