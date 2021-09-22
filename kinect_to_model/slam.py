import open3d as o3d
import sys 
import numpy as np
import matplotlib.pyplot as plt
from openPointCloud import no_rotation
import copy
from datetime import datetime
import os

debug = False
extraDebug = False

def Normalize(V):
    """from ros. Normalizes a vector
    :param V: A vector
    :return: A unit vector pointing in the same direction as z
    Example Input:
        V = np.array([1, 2, 3])
    Output:
        np.array([0.26726124, 0.53452248, 0.80178373])
    """
    return V / np.linalg.norm(V)

def NearZero(z):
    """from ros. Determines whether a scalar is small enough to be treated as zero
    :param z: A scalar input to check
    :return: True if z is close to zero, false otherwise
    Example Input:
        z = -1e-7
    Output:
        True
    """
    return abs(z) < 1e-6

def AxisAng3(expc3):
    """from ros. Converts a 3-vector of exponential coordinates for rotation into
    axis-angle form
    :param expc3: A 3-vector of exponential coordinates for rotation
    :return omghat: A unit rotation axis
    :return theta: The corresponding rotation angle
    Example Input:
        expc3 = np.array([1, 2, 3])
    Output:
        (np.array([0.26726124, 0.53452248, 0.80178373]), 3.7416573867739413)
    """
    return (Normalize(expc3), np.linalg.norm(expc3))

def so3ToVec(so3mat):
    """from ros. Converts an so(3) representation to a 3-vector
    :param so3mat: A 3x3 skew-symmetric matrix
    :return: The 3-vector corresponding to so3mat
    Example Input:
        so3mat = np.array([[ 0, -3,  2],
                           [ 3,  0, -1],
                           [-2,  1,  0]])
    Output:
        np.array([1, 2, 3])
    """
    return np.array([so3mat[2][1], so3mat[0][2], so3mat[1][0]])

def MatrixLog3(R):
    """from ros. Computes the matrix logarithm of a rotation matrix
    :param R: A 3x3 rotation matrix
    :return: The matrix logarithm of R
    Example Input:
        R = np.array([[0, 0, 1],
                      [1, 0, 0],
                      [0, 1, 0]])
    Output:
        np.array([[          0, -1.20919958,  1.20919958],
                  [ 1.20919958,           0, -1.20919958],
                  [-1.20919958,  1.20919958,           0]])
    """
    acosinput = (np.trace(R) - 1) / 2.0
    if acosinput >= 1:
        return np.zeros((3, 3))
    elif acosinput <= -1:
        if not NearZero(1 + R[2][2]):
            omg = (1.0 / np.sqrt(2 * (1 + R[2][2]))) \
                  * np.array([R[0][2], R[1][2], 1 + R[2][2]])
        elif not NearZero(1 + R[1][1]):
            omg = (1.0 / np.sqrt(2 * (1 + R[1][1]))) \
                  * np.array([R[0][1], 1 + R[1][1], R[2][1]])
        else:
            omg = (1.0 / np.sqrt(2 * (1 + R[0][0]))) \
                  * np.array([1 + R[0][0], R[1][0], R[2][0]])
        return VecToso3(np.pi * omg)
    else:
        theta = np.arccos(acosinput)
        return theta / 2.0 / np.sin(theta) * (R - np.array(R).T)

def get_rotation_degree(transformation):
	R = transformation[:3,:3]  # rotation matrix
	so3mat = MatrixLog3(R)
	omg = so3ToVec(so3mat)
	R_dir, theta = AxisAng3(omg) # rotation direction
			# rotation angle (in radians)
	theta_degree = theta / np.pi * 180 # in degree
	return theta_degree

def is_good_transformation(transformation, frame_num, last_translation, last_frame_num):
	######################################
	#####   kick-out rule
	######################################
	#### 1/ rotation is out of plane (5 degree, or 0.087266 in radians);
	#### 2/ too big rotation;
	#### 3/ too big translation;

	if(last_frame_num == 0):
		return True

	# first calculate what is the rotation direction and rotation angle
	tf = transformation
	theta_degree = get_rotation_degree(tf)
	old_theta_degree = get_rotation_degree(last_translation)
	print("new theta degree is ", theta_degree, " old: ", old_theta_degree)
	#angle_with_pl_norm = cal_angle(self.rotation_dir, R_dir)

	trans_tol_x = abs(last_translation[0,3]) + 0.5#0.5  # transformation tolerance, original 0.5
	trans_tol_y = abs(last_translation[1,3]) + 0.5
	trans_tol_z = abs(last_translation[2,3]) + 0.5
	#less than 5 degrees between each two frames is ok, otherwise bad
	rotation_low_tol = old_theta_degree - (5 * (frame_num - last_frame_num)) # original was 30 degrees
	rotation_high_tol = old_theta_degree + (5 * (frame_num - last_frame_num))
	print("rotation_tol low degree is ", rotation_low_tol, " and high is ", rotation_high_tol)
	# angle_with_pl_norm_tol = 0.087266 # in radians (= 5 degrees)
	# angle_with_pl_norm_tol = 0.174533 # in radians (= 10 degrees)
	#angle_with_pl_norm_tol = 0.35 # in radians (= 20 degrees)
	if ( tf[0,3] > trans_tol_x or tf[0,3] < -trans_tol_x or \
		 tf[1,3] > trans_tol_y or tf[1,3] < -trans_tol_y or \
		 tf[2,3] > trans_tol_z or tf[2,3] < -trans_tol_z ):
		#if (debug):
		print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
		# print("here in 1 ")
		print('Something wrong with 1/ translation : (, turn back a little bit...')
		print('>> the translation is [{},{},{}]'.format(tf[0,3],tf[1,3],tf[2,3]))
		return False
	elif (not((theta_degree < rotation_high_tol and theta_degree > rotation_low_tol) or \
		   (theta_degree < - rotation_high_tol and theta_degree > - rotation_low_tol))):
		#if (debug):
		print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
		# print("here in 2 ")
		print('Something wrong with 2/ rotation angle : (, turn back a little bit...')
		print('>> the rotation angle is {} (in degrees)'.format(theta_degree))
		return False
	# elif ( angle_with_pl_norm > angle_with_pl_norm_tol):
	# 	#if(debug):
	# 	print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
	# 	print("here in 3 ")
	# 	print(" angle with pl norm")
	# 	print(angle_with_pl_norm)
	# 	print('Something wrong with 3/ rotation axis : (, turn back a little bit...')
	# 	print('>> the rotation axis is {} (in radians) with plane normal'.format(angle_with_pl_norm))
	# 	return False
	else:
		#if (debug):
		print(">>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
		# print("here in 4 ")
		print("good transformation")
		return True

	def cal_angle(pl_norm, R_dir):
		angle_in_radians = \
			np.arccos(
				np.abs(pl_norm.x*R_dir[0]+ pl_norm.y*R_dir[1] + pl_norm.z*R_dir[2])
				)

		return angle_in_radians

def refine_registration(source, target, source_fpfh, target_fpfh, result_global, voxel_size):
	distance_threshold = voxel_size * 0.4
	if(debug):
		print(":: Point-to-plane ICP registration is applied on original point")
		print("   clouds to refine the alignment. This time we use a strict")
		print("   distance threshold %.3f." % distance_threshold)
	result = o3d.pipelines.registration.registration_icp(
		source, target, distance_threshold, result_global.transformation,
		o3d.pipelines.registration.TransformationEstimationPointToPlane())
	return result

def execute_fast_global_registration(source_down, target_down, source_fpfh,
									 target_fpfh, voxel_size):
	distance_threshold = voxel_size * 0.5
	if(debug):
		print(":: Apply fast global registration with distance threshold %.3f" % distance_threshold)
	result = o3d.pipelines.registration.registration_fast_based_on_feature_matching(
		source_down, target_down, source_fpfh, target_fpfh,
		o3d.pipelines.registration.FastGlobalRegistrationOption(
			maximum_correspondence_distance=distance_threshold))
	return result

def execute_global_registration(source_down, target_down, source_fpfh,
								target_fpfh, voxel_size):
	distance_threshold = voxel_size * 1.5
	if(debug):
		print(":: RANSAC registration on downsampled point clouds.")
		print("   Since the downsampling voxel size is %.3f," % voxel_size)
		print("   we use a liberal distance threshold %.3f." % distance_threshold)
	result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
		source_down, target_down, source_fpfh, target_fpfh, distance_threshold,
		o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
		4, [
			o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
				0.9),
			o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
				distance_threshold)
		], o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500))
	return result


def color_and_depth_to_pointcloud(color_path, depth_path):
	#print(":: upload color and depth and create a PointCloud from them.")
	color_raw = o3d.io.read_image(color_path)
	depth_raw = o3d.io.read_image(depth_path)
	rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(color_raw, depth_raw,convert_rgb_to_intensity = False)

	# plt.subplot(1, 2, 1)
	# plt.title('color image')
	# plt.imshow(rgbd_image.color)
	# plt.subplot(1, 2, 2)
	# plt.title('depth image')
	# plt.imshow(rgbd_image.depth)
	# plt.show()
	#default camera parameter. It has image resolution 640x480, focal length (fx, fy) = (525.0, 525.0),
	# and optical center (cx, cy) = (319.5, 239.5). An identity matrix is used as the default extrinsic parameter
	pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
		rgbd_image,
		o3d.camera.PinholeCameraIntrinsic(o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault))
	# Flip it, otherwise the pointcloud will be upside down
	pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
	#o3d.visualization.draw_geometries([pcd], zoom=0.5)
	#no_rotation(pcd)
	return pcd

def draw_registration_result(source, target, transformation):
	source_temp = copy.deepcopy(source)
	target_temp = copy.deepcopy(target)
	source_temp.paint_uniform_color([1, 0.706, 0])
	target_temp.paint_uniform_color([0, 0.651, 0.929])
	source_temp.transform(transformation)

	o3d.visualization.draw_geometries([source_temp, target_temp],
									  zoom=0.4459,
									  front=[0.9288, -0.2951, -0.2242],
									  lookat=[1.6784, 2.0612, 1.4451],
									  up=[-0.3402, -0.9189, -0.1996])


def preprocess_point_cloud(pcd, voxel_size):
	if(debug):
		print(":: Downsample with a voxel size %.3f." % voxel_size)
	pcd_down = pcd.voxel_down_sample(voxel_size)

	radius_normal = voxel_size * 2
	if(debug):
		print(":: Estimate normal with search radius %.3f." % radius_normal)
	pcd_down.estimate_normals(
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

	radius_feature = voxel_size * 5
	if(debug):
		print(":: Compute FPFH feature with search radius %.3f." % radius_feature)
		#last param is estimated normals
	pcd_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
		pcd_down,
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))
	return pcd_down, pcd_fpfh


def prepare_dataset(source, target, voxel_size):
	if(debug):
		print(":: disturb initial pose of the point clouds.")
	#source = source#o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_0.pcd")
	#target = target#o3d.io.read_point_cloud("../../test_data/ICP/cloud_bin_1.pcd")
	# trans_init = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
	# 						 [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
	# source.transform(trans_init)
	# if(extraDebug):
	# 	draw_registration_result(source, target, np.identity(4))

	source_down, source_fpfh = preprocess_point_cloud(source, voxel_size)
	target_down, target_fpfh = preprocess_point_cloud(target, voxel_size)
	return source_down, target_down, source_fpfh, target_fpfh

def int_to_6_digits(num):
	s_num = str(num)
	for _ in range(0, 6-len(s_num)):
		s_num = "0" + s_num
		#print(s_num)
	return s_num

def ask_user():
	answer = input("accept this addition? y/n ")
	if(answer == "y"):
		return True
	else:
		return False

def register_frame_base(source, target, result_path, voxel_size = 0.05):
	  # means 5cm for the open3d example's dataset. 0.004
	source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)

	#faster ransac variant
	result_fast = execute_fast_global_registration(source_down, target_down,
												   source_fpfh, target_fpfh,
												   voxel_size)
	if(debug):
		print(result_fast)
	if(extraDebug):
		print("after global registration")
		draw_registration_result(source_down, target_down, result_fast.transformation)
		source_temp = copy.deepcopy(source)
		source_temp.transform(result_fast.transformation)
		cloud_temp = target + source_temp
		o3d.visualization.draw_geometries([cloud_temp])
	
	radius_normal = voxel_size * 2
	source.estimate_normals(
	o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
	target.estimate_normals(
	o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

	result_icp = refine_registration(source, target, source_fpfh, target_fpfh, result_fast, voxel_size)
	if(debug):
		print(result_icp)

	print("after icp refinment registration")
	#draw_registration_result(source_down, target_down, result_icp.transformation)
	source_temp = copy.deepcopy(source)
	source_temp.transform(result_icp.transformation)
	cloud_temp = target + source_temp
	#o3d.visualization.draw_geometries([cloud_temp])

	source_trans = copy.deepcopy(source).transform(result_icp.transformation)
	cloud_base = target + source_trans
	# downsampling
	cloud_base = cloud_base.voxel_down_sample(0.005)
	o3d.io.write_point_cloud(result_path, cloud_base)
	return cloud_base

def register_frame_target(source, target, result_path, cloud_base, world_trans, voxel_size = 0.05):
	  # means 5cm for the open3d example's dataset. 0.004
	source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
	#faster ransac variant
	result_fast = execute_fast_global_registration(source_down, target_down,
												   source_fpfh, target_fpfh,
												   voxel_size)
	#print("after global registration - draw registration result")
	#draw_registration_result(source_down, target_down, result_fast.transformation)
	# source_temp = copy.deepcopy(source)
	# source_temp.transform(result_fast.transformation)
	# cloud_temp = target + source_temp
	# print("after global registration - draw geometries of pair")
	# o3d.visualization.draw_geometries([cloud_temp])
	radius_normal = voxel_size * 2
	source.estimate_normals(
	o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
	target.estimate_normals(
	o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))


	tn = datetime.now()
	result_icp = refine_registration(source, target, source_fpfh, target_fpfh, result_fast, voxel_size)
	now = datetime.now()
	print("time that passed doing icp: ", now - tn)
	trans = np.dot(world_trans, result_icp.transformation)
	#print("after icp refinment registration - draw registration result")
	#draw_registration_result(source_down, target_down, result_icp.transformation)
	# source_temp = copy.deepcopy(source)
	# source_temp.transform(trans)
	# cloud_temp = cloud_base + source_temp
	# print("after icp refinment registration - draw geometries all together")
	# o3d.visualization.draw_geometries([cloud_temp])
	source_trans = copy.deepcopy(source).transform(trans)
	tn = datetime.now()
	cloud_base = cloud_base + source_trans
	now = datetime.now()
	print("time that passed adding: ", now - tn)
	# downsampling
	# tn = datetime.now()
	# cloud_base = cloud_base.voxel_down_sample(0.001)#0.01
	# now = datetime.now()
	# print("time that passed down sampling voxels: ", now - tn)
	
	return cloud_base, trans

def lounge_register_1():
	output_path = "C:\\roomie3d\\models\\lounge\\"
	result_path = "C:\\roomie3d\\models\\lounge\\result.ply"
	cloud1 = o3d.io.read_point_cloud(output_path + "result1-52.ply")
	cloud2= o3d.io.read_point_cloud(output_path + "result500-626.ply")
	cloud_base = register_frame_base(cloud1, cloud2, result_path)


def lounge_register_2():
	output_path = "C:\\roomie3d\\models\\lounge\\"
	result_temp_path = "C:\\roomie3d\\models\\lounge\\temp\\"
	result_path = "C:\\roomie3d\\models\\lounge\\result.ply"

	color_only_path = "C:\\roomie3d\\models\\lounge\\color\\"
	depth_only_path = "C:\\roomie3d\\models\\lounge\\depth\\"
	result_path = "C:\\roomie3d\\models\\lounge\\result.ply"

	#create the temp folder. holds the parts of the base cloud,
	#in order to add them all in the end and save runtime
	try:
		os.mkdir(result_temp_path)
	except OSError as error:
		pass
	#remove temp files if exist from previus run
	temp_files = os.listdir(result_temp_path)
	if(len(temp_files) != 0):
		for temp in temp_files:
			try:
				os.remove(result_temp_path+temp)
			except OSError as e:
				print("Error: %s : %s" % (file_path, e.strerror))

	#base_clouds = []
	color1_name = "000001.png"
	depth1_name = "000001.png"
	cloud_base = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
	#base_clouds.append(cloud_base)
	target = copy.deepcopy(cloud_base)
	i = -1
	j = 0
	step = 5
	world_trans = np.identity(4)
	#print(world_trans)
	color_files = os.listdir(color_only_path)
	depth_files = os.listdir(depth_only_path)
	while(j < 3000-step):
		
		j+=1
		#print(j)
		i+=1
		if(i==step):
			i=0
		elif(i!=0):
			continue
		#print(j, "color ", color_files[j])
		source = color_and_depth_to_pointcloud(color_only_path+color_files[j], depth_only_path+depth_files[j])
		#target = color_and_depth_to_pointcloud(color_only_path+color_files[j+1], depth_only_path+depth_files[j+1])
		tn = datetime.now()
		cloud_base, world_trans = register_frame_target(source, target, result_path, cloud_base, world_trans, voxel_size = 0.05)
		now = datetime.now()
		print("time that passed  registering one frame: ", now - tn)
		#print(world_trans)
		#print("end of a pair registration, all together:")
		if(j%100 <= step):
			print(j)
			tn = datetime.now()
			cloud_base = cloud_base.voxel_down_sample(0.005)#0.001
			now = datetime.now()
			print("time that passed down sampling voxels: ", now - tn)
			o3d.io.write_point_cloud(result_temp_path+str(j)+".ply", cloud_base)
			#o3d.visualization.draw_geometries([cloud_base])
			cloud_base = o3d.geometry.PointCloud()
			#base_clouds.append(cloud_base)
		target = copy.deepcopy(source)
	print(j)
	tn = datetime.now()
	cloud_base = cloud_base.voxel_down_sample(0.005)#0.01
	now = datetime.now()
	print("time that passed down sampling voxels: ", now - tn)
	o3d.io.write_point_cloud(result_temp_path+str(j)+".ply", cloud_base)
	#o3d.visualization.draw_geometries([cloud_base])
	cloud_base = o3d.geometry.PointCloud()
	#add all the temp clud bases to one result file
	temp_files = os.listdir(result_temp_path)
	cloud_base = o3d.geometry.PointCloud()
	for temp in temp_files:
		cloud = o3d.io.read_point_cloud(result_temp_path+temp)
		cloud_base = cloud_base + cloud
	o3d.io.write_point_cloud(result_path, cloud_base)
	o3d.visualization.draw_geometries([cloud_base])



def interior_net_dataset_bedroom_registration():
	output_path = "C:\\roomie3d\\models\\3FO4IDE1CLF8_Bedroom\\"
	result_path = "C:\\roomie3d\\models\\3FO4IDE1CLF8_Bedroom\\result.ply"
	color_only_path = "C:\\roomie3d\\models\\3FO4IDE1CLF8_Bedroom\\cam0\\data\\"
	depth_only_path = "C:\\roomie3d\\models\\3FO4IDE1CLF8_Bedroom\\depth0\\data\\"
	
	color1_name = "11.png"
	depth1_name = "11.png"
	order = [10]
	target = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
	cloud_base = target
	for frame in order:
		source = color_and_depth_to_pointcloud(color_only_path+str(frame)+".png", depth_only_path+str(frame)+".png")
		cloud_base = register_frame_base(source, cloud_base, result_path)

def interior_net_dataset_bathroom_registration():
	output_path = "C:\\roomie3d\\models\\media\\binbin\\InteriorNet_HD1\\zip\\3FO4KCUOJKRS\\original_1_1\\"
	result_path = "C:\\roomie3d\\models\\media\\binbin\\InteriorNet_HD1\\zip\\3FO4KCUOJKRS\\original_1_1\\result.ply"
	color_only_path = "C:\\roomie3d\\models\\media\\binbin\\InteriorNet_HD1\\zip\\3FO4KCUOJKRS\\original_1_1\\cam0\\data\\"
	depth_only_path = "C:\\roomie3d\\models\\media\\binbin\\InteriorNet_HD1\\zip\\3FO4KCUOJKRS\\original_1_1\\depth0\\data\\"
	
	color1_name = "0000000000071666664.png"
	depth1_name = "0000000000031666668.png"

	target = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
	cloud_base = target
	i = -1
	j = 0
	world_trans = np.identity(4)
	print(world_trans)
	color_files = os.listdir(color_only_path)
	depth_files = os.listdir(depth_only_path)
	while(j < 1000):
		j+=1
		i+=1
		if(i==5):
			i=0
		elif(i!=0):
			continue
		print(j, "color ", color_files[j])
		source = color_and_depth_to_pointcloud(color_only_path+color_files[j], depth_only_path+depth_files[j])
		target = color_and_depth_to_pointcloud(color_only_path+color_files[j+1], depth_only_path+depth_files[j+1])
		cloud_base, world_trans = register_frame_target(source, target, result_path, cloud_base, world_trans, voxel_size = 0.05)
		print(world_trans)
		print("end of a pair registration")
		o3d.visualization.draw_geometries([cloud_base])
		#cloud_base = target
		






def main():
	#interior_net_dataset_bathroom_registration()
	lounge_register_2()


if __name__ == "__main__":
	main()





def register_lounge_parts_manual():
	color_only_path = "C:\\roomie3d\\models\\lounge\\color\\"
	depth_only_path = "C:\\roomie3d\\models\\lounge\\depth\\"
	result_path = "C:\\roomie3d\\models\\lounge\\result.ply"
	
	color1_name = "000001.png"
	depth1_name = "000001.png"
	cloud_base = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
	t = datetime.now()
	tn = t
	last_translation = 0
	last_frame_num = 0
	step = 10#10
	consecutive_bad = 0
	max_frame = 3000
	parts = [[1,50],[500,626]]#,[51,341],[342,502],[503,632],[633,888],[889,980]]

	i = 2
	while (i <= max_frame):
		print("i is ", str(i))
		#try:
		current_num = int_to_6_digits(i)
		next_num = int_to_6_digits(i+4)

		color2_name = next_num + ".png"
		depth2_name = next_num + ".png"

		target = cloud_base
		source = color_and_depth_to_pointcloud(color_only_path+color2_name, depth_only_path+depth2_name)
		if(debug):
				print("before transform center of source is ", source.get_center())
				print("before transform center of target is ", target.get_center())


		voxel_size = 0.05  # means 5cm for an example dataset. 0.004
		source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)

		#faster ransac variant
		result_fast = execute_fast_global_registration(source_down, target_down,
													   source_fpfh, target_fpfh,
													   voxel_size)
		if(debug):
			print(result_fast)
		if(extraDebug):
			print("after global registration")
			draw_registration_result(source_down, target_down, result_fast.transformation)
			source_temp = copy.deepcopy(source)
			source_temp.transform(result_fast.transformation)
			cloud_temp = target + source_temp
			o3d.visualization.draw_geometries([cloud_temp])
		
		radius_normal = voxel_size * 2
		source.estimate_normals(
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
		target.estimate_normals(
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

		result_icp = refine_registration(source, target, source_fpfh, target_fpfh, result_fast, voxel_size)
		if(debug):
			print(result_icp)
		
		print("after icp refinment registration")
		#draw_registration_result(source_down, target_down, result_icp.transformation)
		source_temp = copy.deepcopy(source)
		source_temp.transform(result_icp.transformation)
		cloud_temp = target + source_temp
		o3d.visualization.draw_geometries([cloud_temp])


		#if the transformation is good, use it
		if(True):#ask_user()):#is_good_transformation(result_icp.transformation, i, last_translation, last_frame_num)):
			consecutive_bad = 0
			source.transform(result_icp.transformation)
			cloud_base = cloud_base + source
			if(debug):
				print("after plus center is ", cloud_base.get_center())
			# downsampling
			cloud_base = cloud_base.voxel_down_sample(0.001)
			o3d.io.write_point_cloud(result_path, cloud_base)
			#print(i)
			#if(i % 21 == 0):
			print("we are in frame ", i)
			now = datetime.now()
			print("time that passed from last iteration: ", now - tn)
			tn = now
			print("time that passed from start: ", now - t)

			#o3d.visualization.draw_geometries([cloud_base])
			o3d.io.write_point_cloud(depth_only_path+str(i)+".ply", cloud_base)
			last_frame_num = i
			last_translation = result_icp.transformation
			i += step
		else:
			#move faster if has consecutive bad registration attemts
			print("frame ", str(i), " not used")
			consecutive_bad += 1
			if(consecutive_bad > 2):
				i += 50
			else:
				i += step
			print("i is ", str(i))
		if(i > max_frame):
			i = max_frame

	o3d.visualization.draw_geometries([cloud_base])

def register_lounge_all_at_once():
	posWorldTrans = np.identity(4)
	#posWorldTrans = np.asarray([[0.0, 0.0, 1.0, 0.0], [1.0, 0.0, 0.0, 0.0],
	#						 [0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]])
	color_only_path = "C:\\roomie3d\\models\\lounge\\color\\"
	depth_only_path = "C:\\roomie3d\\models\\lounge\\depth\\"
	result_path = "C:\\roomie3d\\models\\lounge\\result.ply"
	
	color1_name = "000002.png"
	depth1_name = "000002.png"
	cloud_base = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
	t = datetime.now()
	tn=t
	last_translation = 0
	last_frame_num = 0
	step = 10
	consecutive_bad = 0
	max_frame = 3000
	# if(debug):
	# 	print("cloud base's center is ", cloud_base.get_center())
	# o3d.io.write_point_cloud(result_path, cloud_base)

	#for i in range (1,3000,step):#3000
	i = 1
	while (i <= max_frame):
		print("i is ", str(i))
		#try:
		current_num = int_to_6_digits(i)
		next_num = int_to_6_digits(i+4)
		#color1_name = current_num + ".png"
		#depth1_name = current_num + ".png"
		color2_name = next_num + ".png"
		depth2_name = next_num + ".png"
		#color2_name = "000005.png"
		#depth2_name = "000005.png"
		#target = color_and_depth_to_pointcloud(color_only_path+color1_name, depth_only_path+depth1_name)
		target = cloud_base
		#cloud_base = target
		source = color_and_depth_to_pointcloud(color_only_path+color2_name, depth_only_path+depth2_name)
		#o3d.visualization.draw_geometries([source+target])
		#target = cloud_base
		if(debug):
				print("before transform center of source is ", source.get_center())
				print("before transform center of target is ", target.get_center())

		voxel_size = 0.05  # means 5cm for an example dataset. 0.004
		source_down, target_down, source_fpfh, target_fpfh = prepare_dataset(source, target, voxel_size)
		#slower ransac variant
		# result_ransac = execute_global_registration(source_down, target_down,
		# 											source_fpfh, target_fpfh,
		# 											voxel_size)
		# print(result_ransac)
		# if(debug):
		# 	draw_registration_result(source_down, target_down, result_ransac.transformation)
		# result_icp = refine_registration(source_down, target_down, source_fpfh, target_fpfh, result_ransac,
		# 							 voxel_size)
		# print(result_icp)
		# if(debug):
		# 	draw_registration_result(source_down, target_down, result_icp.transformation)

		#faster ransac variant
		result_fast = execute_fast_global_registration(source_down, target_down,
													   source_fpfh, target_fpfh,
													   voxel_size)
		#print("Fast global registration took %.3f sec.\n" % (time.time() - start))
		if(debug):
			print(result_fast)
		if(extraDebug):
			print("after global registration")
			draw_registration_result(source_down, target_down, result_fast.transformation)
			source_temp = copy.deepcopy(source)
			source_temp.transform(result_fast.transformation)
			cloud_temp = target + source_temp
			o3d.visualization.draw_geometries([cloud_temp])
		
		radius_normal = voxel_size * 2
		source.estimate_normals(
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
		target.estimate_normals(
		o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

		result_icp = refine_registration(source, target, source_fpfh, target_fpfh, result_fast, voxel_size)
		if(debug):
			print(result_icp)
		
		print("after icp refinment registration")
		#draw_registration_result(source_down, target_down, result_icp.transformation)
		source_temp = copy.deepcopy(source)
		source_temp.transform(result_icp.transformation)
		cloud_temp = target + source_temp
		o3d.visualization.draw_geometries([cloud_temp])

		#if the transformation is good, use it
		if(ask_user()):#is_good_transformation(result_icp.transformation, i, last_translation, last_frame_num)):
			# #move from local to world
			# if(debug):
			# 	print("local: ", result_icp.transformation)
			# 	print("world: ", posWorldTrans)
			# posWorldTrans =  np.dot(posWorldTrans, result_icp.transformation)
			# if(debug):
			# 	print("world after world X local: ", posWorldTrans)
			# #use the transformation on the source and add to the result cloud 
			# source.transform(posWorldTrans)
			# #try adding 120 
			# if(debug):
			# 	print("after transform center is ", source.get_center())
			# #source.transform(result_icp.transformation)
			consecutive_bad = 0
			source.transform(result_icp.transformation)
			cloud_base = cloud_base + source
			if(debug):
				print("after plus center is ", cloud_base.get_center())
			# downsampling
			cloud_base = cloud_base.voxel_down_sample(0.001)
			o3d.io.write_point_cloud(result_path, cloud_base)
			#print(i)
			#if(i % 21 == 0):
			print("we are in frame ", i)
			now = datetime.now()
			print("time that passed from last iteration: ", now - tn)
			tn = now
			print("time that passed from start: ", now - t)

			#o3d.visualization.draw_geometries([cloud_base])
			o3d.io.write_point_cloud(depth_only_path+str(i)+".ply", cloud_base)
			last_frame_num = i
			last_translation = result_icp.transformation
			i += step
		else:
			#move faster if has consecutive bad registration attemts
			print("frame ", str(i), " not used")
			consecutive_bad += 1
			if(consecutive_bad > 2):
				i += 50
			else:
				i += step
			print("i is ", str(i))
		if(i > max_frame):
			i = max_frame
			
			
		# except:
		# 	print("exception in ", i)
		#if(debug):
	o3d.visualization.draw_geometries([cloud_base])