#!/usr/bin/env python3

"""
NOTE: DONT FORGET TO INSTALL:
pip install trimesh
pip install python-fcl
pip install rtree
pip install open3d # ~300MB ,needed for obj. simplification!
"""

import rospy

import visualization_msgs.msg # Marker
import geometry_msgs.msg # Point32
import std_msgs.msg  # Float32

import trimesh # To read obj and find the distance between the obj file and the fabric mesh
import numpy as np

import time 


class FabricToPolyhedronDistance:
    def __init__(self):
        rospy.init_node('fabric_to_polyhedron_distance', anonymous=False)

        self.fabric_marker_topic_name = rospy.get_param("~fabric_marker_topic_name", "/fabric_points")
        self.fabric_face_tri_ids_topic_name = rospy.get_param("~fabric_face_tri_ids_topic_name", "/fabric_face_tri_ids")

        self.pub_distance_topic_name = rospy.get_param("~pub_distance_topic_name", "/distance_to_obj")

        self.pub_rate_polyhedron = rospy.get_param("~pub_rate_polyhedron", 1)
        self.pub_rate_min_distance = rospy.get_param("~pub_rate_min_distance", 50)
        
        self.viz_polyhedron_topic_name = rospy.get_param("~viz_polyhedron_topic_name", "/polyhedron")
        self.viz_min_distance_line_topic_name = rospy.get_param("~viz_min_distance_line_topic_name", "/min_distance_line_marker")
        
        self.polyhedron_model_obj_file_path = rospy.get_param("~polyhedron_model_obj_file_path")

        self.polyhedron_publish = rospy.get_param("~polyhedron_publish", True)

        self.viz_min_distance_line_color_rgba = rospy.get_param("~viz_min_distance_line_color_rgba", [1.0, 1.0, 1.0, 1.0])
        self.viz_polyhedron_color_rgba        = rospy.get_param("~viz_polyhedron_color_rgba",        [1.0, 1.0, 1.0, 1.0])
        
        self.polyhedron_translation   = np.array(rospy.get_param("~polyhedron_translation",   [0.0, 0.0, 0.0]))
        self.polyhedron_rotationAxis  = np.array(rospy.get_param("~polyhedron_rotationAxis",  [0, 0, 1]))
        self.polyhedron_rotationAngle = np.deg2rad(np.array(rospy.get_param("~polyhedron_rotationAngle", 0.0)))
        self.polyhedron_scale         = np.array(rospy.get_param("~polyhedron_scale",         [1, 1, 1]))
        

        # Load polyhedron from obj file
        self.polyhedron_mesh = trimesh.load_mesh(self.polyhedron_model_obj_file_path)

        self.max_polyhedron_faces = rospy.get_param("~max_polyhedron_faces", 300)
        rospy.loginfo("Loaded mesh has: " + str(len(self.polyhedron_mesh.faces)) + " faces.")

        if len(self.polyhedron_mesh.faces) > self.max_polyhedron_faces:
            # Check the number of faces in the original mesh
            rospy.logwarn("The loaded mesh has faces more than specified max faces =" + str(self.max_polyhedron_faces) + ".")
            rospy.logwarn("The loaded mesh will be simplified accordingly.")

            # Perform quadric mesh simplification
            self.polyhedron_mesh = self.polyhedron_mesh.simplify_quadric_decimation(self.max_polyhedron_faces)
            rospy.logwarn("Loaded mesh has: " + str(len(self.polyhedron_mesh.faces)) + " faces after simplification.")

        if not self.polyhedron_mesh.is_watertight:
            rospy.logwarn('Warning: polyhedron_mesh is not closed, signed_distance may not be accurate')

        # Translate rotate scale the polyhedron 
        self.polyhedron_mesh.apply_scale(self.polyhedron_scale)

        rotation_matrix = trimesh.transformations.rotation_matrix(self.polyhedron_rotationAngle, self.polyhedron_rotationAxis)
        self.polyhedron_mesh.apply_transform(rotation_matrix)

        self.polyhedron_mesh.apply_translation(self.polyhedron_translation)

        # Create a proximity query object for each mesh
        self.proximity_query_polyhedron = trimesh.proximity.ProximityQuery(self.polyhedron_mesh)
        
        
        # initialize fabric data structure
        self.face_tri_ids = None
        self.vertices = None

        rospy.Subscriber(self.fabric_marker_topic_name, 
                         visualization_msgs.msg.Marker, 
                         self.fabric_marker_sub_callback, 
                         queue_size=10)
        
        # rospy.Subscriber(self.fabric_face_tri_ids_topic_name, 
        #                  std_msgs.msg.Int32MultiArray, 
        #                  self.fabric_face_tri_ids_sub_callback, 
        #                  queue_size=1)

        self.pub_distance          = rospy.Publisher(self.pub_distance_topic_name, std_msgs.msg.Float32, queue_size=10)
        self.pub_min_distance_line = rospy.Publisher(self.viz_min_distance_line_topic_name, visualization_msgs.msg.Marker, queue_size=10)

        if self.polyhedron_publish:
            self.pub_polyhedron = rospy.Publisher(self.viz_polyhedron_topic_name, visualization_msgs.msg.Marker, queue_size=1)
            self.pub_polyhedron_wireframe = rospy.Publisher(self.viz_polyhedron_topic_name + "_wireframe", visualization_msgs.msg.Marker, queue_size=1)

            self.polyhedron_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_polyhedron), self.polyhedron_pub_timer_callback)
        
        self.min_distance_pub_timer = rospy.Timer(rospy.Duration(1. / self.pub_rate_min_distance), self.min_distance_pub_timer_callback)
        
    """
    def fabric_face_tri_ids_sub_callback(self, msg):
        if self.face_tri_ids is None:
            # every three integers represent the indices of a single triangle's vertices
            self.face_tri_ids = np.array(msg.data).reshape((-1, 3))
            # print(self.face_tri_ids)
    """


    def fabric_marker_sub_callback(self, msg):
        # check if the marker is a point list 
        if msg.type == visualization_msgs.msg.Marker.POINTS:
            self.vertices = np.array([(p.x, p.y, p.z) for p in msg.points])
            

    def min_distance_pub_timer_callback(self,event):
        # # if self.face_tri_ids is not None:
        if self.vertices is not None:
            """
            # ------------------------ METHOD 1: SIGNED DISTANCES ------------------------
            # # Use signed_distance to compute the signed distances from fabric vertices to the polyhedron
            # distances = -self.proximity_query_polyhedron.signed_distance(self.vertices) # NOTE: Creates a bottleneck!
            # # distances = -trimesh.proximity.signed_distance(self.polyhedron_mesh, self.vertices) # NOTE: Creates a bottleneck!

            # # Find the minimum element in the array
            # min_distance = np.min(distances)

            # # Find the index of the minimum element in the array
            # min_distance_index = np.argmin(distances)

            # point_on_fabric = self.vertices[min_distance_index]
            
            # # Find the corresponding point on the surface of the polyhedron
            # closest_point, _, _ = self.proximity_query_polyhedron.on_surface(point_on_fabric.reshape(1, 3))

            # point_on_polyhedron = closest_point.squeeze()

            # ------------------------ METHOD 2: ONLY POSITIVE DISTANCES ------------------------
            # closest_points, distances, _ = trimesh.proximity.closest_point(self.polyhedron_mesh, self.vertices)

            # # Find the minimum element in the array
            # min_distance = np.min(distances)

            # # Find the index of the minimum element in the array
            # min_distance_index = np.argmin(distances)

            # point_on_fabric = self.vertices[min_distance_index]
            # point_on_polyhedron = closest_points[min_distance_index].squeeze()
            """

            # ------------------------ METHOD 3: ONLY POSITIVE DISTANCES ------------------------
            # init_t = time.time()
            
            closest_points, distances, _ = self.polyhedron_mesh.nearest.on_surface(self.vertices)

            # Find the minimum element in the array
            min_distance = np.min(distances)

            # rospy.logwarn("Fabric distance calculation time: " + str(1000*(time.time() - init_t)) + " ms.")

            # Find the index of the minimum element in the array
            min_distance_index = np.argmin(distances)

            point_on_fabric = self.vertices[min_distance_index]
            point_on_polyhedron = closest_points[min_distance_index].squeeze()


            # ------------------------ RESULTS ------------------------
            # # print("min_distance: ",min_distance)
            # # print("closest_point on fabric: ",     point_on_fabric)
            # # print("closest_point on polyhedron: ", point_on_polyhedron)    
            
            self.publish_min_distance_line_marker(point_on_fabric, point_on_polyhedron)
            self.pub_distance.publish(std_msgs.msg.Float32(data=min_distance))


    def polyhedron_pub_timer_callback(self, event):
        # Create Polyhedron Marker message
        marker = visualization_msgs.msg.Marker()
        
        marker.header.frame_id = "map"
        marker.type = marker.TRIANGLE_LIST
        marker.action = marker.ADD
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        # Set the marker orientation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Set the marker color
        marker.color.r = self.viz_polyhedron_color_rgba[0]
        marker.color.g = self.viz_polyhedron_color_rgba[1]
        marker.color.b = self.viz_polyhedron_color_rgba[2]
        marker.color.a = self.viz_polyhedron_color_rgba[3]

        for face in self.polyhedron_mesh.faces:
            triangle_points = self.polyhedron_mesh.vertices[face]
            for point in triangle_points:
                p = geometry_msgs.msg.Point()
                p.x, p.y, p.z = point
                marker.points.append(p)

        # Publish the Polyhedron Marker message
        self.pub_polyhedron.publish(marker)

        # # Create a new Marker message for the wireframe
        wireframe_marker = visualization_msgs.msg.Marker()
        wireframe_marker.header.frame_id = "map"
        wireframe_marker.type = wireframe_marker.LINE_LIST
        wireframe_marker.action = wireframe_marker.ADD
        wireframe_marker.scale.x = 0.01 # Choose a suitable line width
        wireframe_marker.pose.orientation.w = 1.0
        
        # Set color for wireframe
        wireframe_marker.color.r = 0.5
        wireframe_marker.color.g = 0.5
        wireframe_marker.color.b = 0.5
        wireframe_marker.color.a = 0.9
        
        for face in self.polyhedron_mesh.faces:
            triangle_points = self.polyhedron_mesh.vertices[face]
            for i in range(3):
                p1 = geometry_msgs.msg.Point()
                p1.x, p1.y, p1.z = triangle_points[i]
                
                p2 = geometry_msgs.msg.Point()
                p2.x, p2.y, p2.z = triangle_points[(i+1) % 3]
                
                wireframe_marker.points.append(p1)
                wireframe_marker.points.append(p2)
        
        # Publish the wireframe Marker message
        self.pub_polyhedron_wireframe.publish(wireframe_marker)

    def publish_min_distance_line_marker(self, point_on_fabric, point_on_polyhedron):
        # Prepare a Marker message for the shortest distance
        min_distance_marker = visualization_msgs.msg.Marker()
        min_distance_marker.header.frame_id = "map"
        
        min_distance_marker.type = visualization_msgs.msg.Marker.LINE_STRIP
        min_distance_marker.action = visualization_msgs.msg.Marker.ADD

        min_distance_marker.pose.orientation.w = 1.0;

        min_distance_marker.scale.x = 0.01 # specify a suitable size
        r = self.viz_min_distance_line_color_rgba[0]
        g = self.viz_min_distance_line_color_rgba[1]
        b = self.viz_min_distance_line_color_rgba[2]
        a = self.viz_min_distance_line_color_rgba[3]
        min_distance_marker.color = std_msgs.msg.ColorRGBA(r=r, g=g, b=b, a=a) # Red color
        min_distance_marker.points.append(geometry_msgs.msg.Point(x=point_on_fabric[0], y=point_on_fabric[1], z=point_on_fabric[2]))
        min_distance_marker.points.append(geometry_msgs.msg.Point(x=point_on_polyhedron[0], y=point_on_polyhedron[1], z=point_on_polyhedron[2]))

        self.pub_min_distance_line.publish(min_distance_marker)

if __name__ == "__main__":
    fabricToPolyhedronDistance = FabricToPolyhedronDistance()
    rospy.spin()
