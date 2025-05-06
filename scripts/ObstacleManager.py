import math
import numpy as np
import rospy
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point

#-------------------------------------------------------------------------#
# Class: ObstacleManager

# Objective: 
# We will create a class whose purpose will be to deal with
# all the obstacles in the arena. The idea will be to classify the objects
# found by their size (we will call it length here because what really is
# important to us is the length of the object - we do not want to classify
# the wall as an obstacle). Only the obstacles that are smaller than the 
# lenth threshold will be classified as obstacles, the rest will remain as 
# clusters

#-------------------------------------------------------------------------#
class ObstacleManager:

    # Constructor
    def __init__(self, obstacle_threshold=0.5, proximity_threshold=0.1, length_threshold=0.40, clusters_distance_threshold = 0.1, obstacles_distance_threshold = 0.2):
        
        #Variables

        # Obstacles and clusters lists
        self.close_obstacles = []
        self.obstacles = []
        self.clusters = []

        # Thresholds
        self.obstacle_threshold = obstacle_threshold
        self.proximity_threshold = proximity_threshold
        self.length_threshold = length_threshold
        self.clusters_distance_threshold = clusters_distance_threshold
        self.obstacles_distance_threshold = obstacles_distance_threshold

        # Robot's current position and orientations
        self.robot_position = [0, 0]
        self.robot_orientation = 0

        # Publishers we will use to visualize the clusters and obstacles in rviz
        self.marker_pub_clusters = rospy.Publisher('/visualization_clusters', MarkerArray, queue_size=10)
        self.marker_pub_obstacles = rospy.Publisher('/visualization_obstacles', MarkerArray, queue_size=10)


    # Method that will use the data from the lidar to map all the clusters and obstacles found in the arena
    def update_obstacles(self, scan):

        # We first collect all the points found around the robot that are closer than the obstacle_threshold
        temp_points = self._collect_points(scan)

        # We append the points that are close to the clusters we already have or we create new clusters
        self._cluster_points(temp_points)

        # We merge the clusters that are too close to each other to avoid having multiple clusters for the wall
        self._merge_clusters()

        # We filter the clusters we found and add the object smaller than length_threshold to the obstacles list
        self._filter_and_update_obstacles(scan)

        # We publish the points found for rviz
        self.publish_clusters()
        self.publish_obstacles()


    # Method that will collect all the points found around the robot that are closer than the obstacle_threshold
    def _collect_points(self, scan):
        points = []
        for i, distance in enumerate(scan.ranges):
            if 0 < distance < self.obstacle_threshold:
                angle = math.degrees(i * scan.angle_increment)
                point = self.get_global_coordinates(distance, angle)
                points.append(point)

        return points

    # Method that will append the points that are close to the clusters we already have or 
    # create new clusters

    def _cluster_points(self, points):
        for point in points:
            for cluster in self.clusters:
                if any(self.calculer_distance(p, point) < self.proximity_threshold for p in cluster):
                    cluster.append(point)
                    break
            else:
                self.clusters.append([point])
    

    # Method that will merge the clusters that are too close to each other to avoid having 
    # multiple clusters for the wall

    def _merge_clusters(self):
        changed = True
        while changed:
            changed = False
            merged = [False] * len(self.clusters)
            new_clusters = []

            for i, cluster1 in enumerate(self.clusters):
                if not merged[i]:
                    new_cluster = cluster1[:]
                    merged[i] = True

                    for j in range(len(self.clusters)):
                        if not merged[j] and j != i:
                            if any(self.calculer_distance(point1, point2) < self.clusters_distance_threshold for point1 in new_cluster for point2 in self.clusters[j]):
                                new_cluster.extend(self.clusters[j])  # Merge clusters
                                merged[j] = True
                                changed = True  # A merge occurred, so a new pass is needed

                    new_clusters.append(new_cluster)

            self.clusters = new_clusters


    # Method that filters the clusters we found and adds the objects smaller than length_threshold
    # to the obstacles list
    def _filter_and_update_obstacles(self, scan):
        processed_clusters = []
        for cluster in self.clusters:
            if len(cluster) > 1:
                processed_clusters.append(self._process_cluster(cluster, scan))
        self.processed_clusters = processed_clusters
        close_obstacles = []
        for cluster in processed_clusters:
            if cluster['length'] < self.length_threshold:
                close_obstacles.append(cluster)
        self.obstacles = close_obstacles
        self.sort_obstacles()


    # Method that will process the clusters we find, adding them to a dictionary along with the coordinates
    # of their centroid and their 'lenth'
    def _process_cluster(self, cluster, scan):
        xs, ys = zip(*cluster)
        centroid = (np.mean(xs), np.mean(ys))
        length = max(xs) - min(xs) + max(ys) - min(ys)
        #cote = self.get_cote_obstacle(scan, centroid, length)
        return {'centroid': centroid, 'length': length}
    

    # Method that will process the clusters we find, adding them to a dictionary along with the coordinates
    # of their centroid and their 'lenth'
    def _process_cluster(self, cluster, scan):
        xs, ys = zip(*cluster)
        centroid = (np.mean(xs), np.mean(ys))
        length = max(xs) - min(xs) + max(ys) - min(ys)
        return {'centroid': centroid, 'length': length}


    # Method that will sort the obstacles in the vecinity by their distance to the robot
    def sort_obstacles(self):
        self.obstacles.sort(key=lambda x: self.distance_to_obstacle(x))


    # Method that will update the robot's current position using the data we will get from the topic /odom
    def update_robot_position(self, position, orientation):
        self.robot_position = position
        self.robot_orientation = orientation


    # Method that will calculate the real coordinates of an object using the robot position and orientation
    # and the distance and angle to the actual object
    def get_global_coordinates(self, distance, angle):
        x = self.robot_position[0] + distance * math.cos(math.radians(self.robot_orientation + angle))
        y = self.robot_position[1] + distance * math.sin(math.radians(self.robot_orientation + angle))
        return (x, y)


    # Method that will use the robot's current orientation and position to calculate it's relative
    # angle to the point in the parameter
    def calculate_relative_angle(self, point):
        theta_radians = math.radians(self.robot_orientation)
        angle_to_target = math.atan2(point[1] - self.robot_position[1], point[0] - self.robot_position[0])
        relative_angle = (angle_to_target - theta_radians + math.pi) % (2 * math.pi) - math.pi
        return math.degrees(relative_angle)


    # Method that will calculate the Euclidian distance between two points in space
    def calculer_distance(self, p1, p2):
        return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

                            
    # Method used to determine the side of an object according to its coordinates on the x axis relative
    # to the center between the first two obstacles. Since we know the obstacles are placed on the y axis
    # of the /odom topic, we can determine that if an obstacle is closer to the x coordinate 0 than the
    # center between the first two obstacles(we keep choose to calculate this center only once to avoid
    # errors based on the points we find), it means that this obstacle is on the left side of the track and 
    # if it's farther away it means it's on the right.
            
    def get_cote_obstacle(self, center):

        cote = "we don't know"
        if len(self.obstacles)>1:
            if self.obstacles[0]['centroid'][0] - center[0] < 0:
                cote = "left"
            else:
                cote = "right"
        else:
            cote = "we don't know"
        return cote
    

    # Same method but for the real robot since it's a different axis
    def get_cote_obstacle_reel(self, center):

        cote = "we don't know"
        if len(self.obstacles)>1:
            if self.obstacles[0]['centroid'][1] - center[1] < 0:
                cote = "left"
            else:
                cote = "right"
        else:
            cote = "we don't know"
        return cote
    

    # Method that will measure the theoretical distance between the robot and an obstacle we put in the 
    # parameters using the data from the /odom topic. We say 'theoretical' because since the /odom topic
    # tends to drift a bit in relation to the real coordinates, the distance we get with this method is 
    # not the most accurate and needs to be used with care
    def distance_to_obstacle(self, obstacle):
        return self.calculer_distance(obstacle['centroid'], self.robot_position)
    


    # Method that will calculate the coordinates of the center between the first two obstacles closest to the robot
    def get_milieu_entre_obstacles(self):
        return ((self.obstacles[0]['centroid'][0]+self.obstacles[1]['centroid'][0])/2, (self.obstacles[0]['centroid'][1] + self.obstacles[1]['centroid'][1])/2)


    # Methods that will publish the clusters and obstacles markers for rviz
    def publish_clusters(self):
        marker_array = MarkerArray()
        # Clear old markers first
        for i in range(len(self.clusters)):
            delete_marker = Marker()
            delete_marker.header.frame_id = "odom"
            delete_marker.id = i
            delete_marker.action = Marker.DELETE
            marker_array.markers.append(delete_marker)

        self.marker_pub_clusters.publish(marker_array)

        # Add new/updated markers
        marker_array = MarkerArray()
        for i, cluster in enumerate(self.clusters):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.POINTS
            marker.action = marker.ADD
            marker.id = i
            marker.scale.x = 0.05
            marker.scale.y = 0.05
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0

            for point in cluster:
                p = Point()
                p.x, p.y = point[0], point[1]
                p.z = 0
                marker.points.append(p)

            marker_array.markers.append(marker)

        self.marker_pub_clusters.publish(marker_array)


    def publish_obstacles(self):
        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = delete_marker.DELETEALL
        marker_array.markers.append(delete_marker)
        self.marker_pub_obstacles.publish(marker_array)

        for i, ob in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.id = i
            marker.scale.x = 0.1  # Sphere radius
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.pose.position.x = ob['centroid'][0]
            marker.pose.position.y = ob['centroid'][1]
            marker.pose.position.z = 0

            marker_array.markers.append(marker)

        self.marker_pub_obstacles.publish(marker_array)


    