import cv2 as cv
import networkx as nx
import numpy as np
import threading
import queue
from util import UtilityFunctions as uf
from graph import Graph as gr


def main():
    video = "img/video/test_with_block.mov"
    vg = VideoToGraph(75, 150, video)
    vg.start_environment()
    vg.tear_down()

class VideoToGraph:

    #initilaize
    def __init__(self, width, length, video_file, metric = True):

        # video feed
        self.cap = self.initialize_camera(video_file)

        # Graph setup
        self.maze_height = width if metric else width * 2.54
        self.maze_length = length if metric else length * 2.54
        self.grid_height = 0
        self.grid_width = 0
        self.corners = {}
        self.matrix = any
        self.graph = nx.Graph()

        # shortest paths from robot to goal
        self.paths = {}
        self.robot_goals = {}

        # QR Code / robot tracking 
        self.qcd = None
        self.overlapping_nodes = set()
        self.tracked_objects = {}

        # relay processed video via thread to avoid creating a blocking call
        self.frame_queue = queue.Queue(maxsize=1)
        self.running = True
        self.thread = threading.Thread(target=self.start_environment, daemon=True)
        self.thread.start()


    # Video input
    def initialize_camera(self, camera = int(0)):
        capture = cv.VideoCapture(camera) # 0 is the default camera, can also take a file

        if not capture.isOpened():
            print("Cannot open camera")
            input("Press Enter to exit... ")
            exit()

        return capture
    
    # Release the camera 
    def tear_down(self):
        self.running = False
        self.thread.join()
        self.cap.release()
        cv.destroyAllWindows()

    # Create and update graph from the video input
    def start_environment(self, overlay_update_frame_interval=40):
        frame_count = 0  # Count frames to update the overlay after a set number of frames
        refresh_graph = True  
        while self.running:
            # Capture frame-by-frame
            ret, frame = self.cap.read()
            # if frame is read correctly ret is True
            if not ret:
                print("Can't receive frame (stream end?). Exiting ...")
                break

            refresh_graph = True if frame_count % overlay_update_frame_interval*3 == 0 else False
            update = frame_count % overlay_update_frame_interval == 0
            if update:
                graph = self.convert_image_to_graph(frame, refresh_graph)
                self.detect_objects(frame)
                refresh_graph = False

            overlay_image = self.draw_overlay(frame, graph)
            self.draw_objects_overlay(overlay_image)
            try:
                paths = self.find_paths(self.robot_goals)
                for robot_path in paths.keys():
                    if robot_path:
                        path = paths[robot_path]
                        overlay_image = gr.draw_transformed_path(overlay_image, graph, path)
                        self.paths[robot_path] = path
            except:
                if update:
                    print("Couldn't find path")

            # Display the (frame + overlay)
            # cv.imshow('frame_with_overlay', overlay_image)
            if not self.frame_queue.full():
                self.frame_queue.put(overlay_image)
            frame_count += 1
    
    def convert_image_to_graph(self, image, refresh_graph):
        if refresh_graph:
            corners = uf.find_corners(image)
            if self.corners != corners:
                print("updating corners")
                self.corners = corners
                self.set_dimensions(corners)
                self.graph = nx.grid_2d_graph(self.grid_width, self.grid_height)
                gr.add_diagonal_edges(self.grid_width, self.grid_height, self.graph)
                self.refresh_matrix(corners)
                gr.set_node_positions(self.graph, self.matrix)
                self.set_robot_goals({
                    'robot 1': (self.grid_width - 1, self.grid_height - 4),
                    'robot 2': (self.grid_width - 3, self.grid_height - 12),          
                    })
            self.detect_static_obstacles(image, self.graph)
            self.detect_objects(image)
            gr.adjust_graph_weights(self.graph)        

        return self.graph
    
    def refresh_matrix(self, corners):
        matrix = uf.compute_affine_transformation(corners, self.grid_width, self.grid_height)
        self.matrix = matrix

    def draw_overlay(self, image, graph):
        overlay_image = gr.draw_nodes_overlay(graph, image)
        overlay_image = gr.draw_edges_overlay(graph, overlay_image)
        return overlay_image

    def draw_objects_overlay(self, overlay_image):
        for key in self.tracked_objects.keys():
            pts = self.tracked_objects[key].astype(int)
            cv.polylines(overlay_image, [pts], isClosed=True, color=uf.GREEN, thickness=2)
            cv.putText(overlay_image, key, (pts[0][0]+20, pts[0][1]-20), cv.FONT_HERSHEY_SIMPLEX, 1.3, (0,0,0), 3)
    
    def find_paths(self, robot_goal):
        paths = {}
        for robot, goal in robot_goal.items():
            robot_position = self.tracked_objects[robot]
            center = uf.find_center_of_rectangle(robot_position)
            path = gr.a_star_from_pixel_pos(self.graph, center, goal)
            paths[robot] = path
        return paths

    def set_robot_goals(self, goals):
        for robot, goal in goals.items():
            self.robot_goals[robot] = goal

    def set_dimensions(self, corners, block_size_cm=3.5):
        # Compute grid dimensions based on the block size and image size
        image_width_px = corners['top_right'][0] - corners['top_left'][0]
        image_height_px = corners['bottom_left'][1] - corners['top_left'][1]

        pixel_block_height_px  = (block_size_cm / self.maze_height) * image_height_px
        pixel_block_width_px = (block_size_cm / self.maze_length) * image_width_px

        self.grid_width = int(image_width_px / pixel_block_width_px)
        self.grid_height = int(image_height_px / pixel_block_height_px)         
    
    def detect_static_obstacles(self, image, graph, proximity_threshold=60):
        overlay_image = image.copy()
        hsv_image = cv.cvtColor(overlay_image, cv.COLOR_BGR2HSV)
        pink_lower = [140, 50, 50]
        pink_upper = [170, 255, 255]
        contours = uf.find_contours(hsv_image, pink_lower, pink_upper)

        MIN_CONTOUR_AREA = 3000
        filtered_contours = [cnt for cnt in contours if cv.contourArea(cnt) > MIN_CONTOUR_AREA]

        for contour in filtered_contours:
            cv.drawContours(overlay_image, [contour], -1, uf.RED, 2)
            gr.update_graph_based_on_obstacle(graph, contour, proximity_threshold)

    def detect_objects(self, image):
        # Initialize the QR code detector
        self.qcd = cv.QRCodeDetector() if self.qcd is None else self.qcd
        
        # Detect and decode QR codes
        retval, decoded_infos, points, _ = self.qcd.detectAndDecodeMulti(image)
        
        if retval:
            for i, qr_code_info in enumerate(decoded_infos):
                self.update_position(points[i], qr_code_info)

        for key in self.tracked_objects.keys():
            try:
                if key[0] == "a":
                    qr_code_points = self.tracked_objects[key].astype(int)
                    overlapping_nodes = self.check_qr_code_overlap(self.graph, qr_code_points)
                    gr.update_graph_based_on_qr_code(self.graph, overlapping_nodes, self.overlapping_nodes)
                    self.overlapping_nodes = overlapping_nodes
            except:
                print(f"Invalid QR code detected: {key}")
    
    def check_qr_code_overlap(self, graph, qr_code_points, proximity_threshold=25):
        # Get the bounding box of the QR code in graph space
        min_x = min([pt[0] for pt in qr_code_points])
        max_x = max([pt[0] for pt in qr_code_points])
        min_y = min([pt[1] for pt in qr_code_points])
        max_y = max([pt[1] for pt in qr_code_points])
        
        # Check for nodes within the bounding box
        overlapping_nodes = gr.find_nodes_within_bounding_box(graph, min_x, max_x, min_y, max_y, proximity_threshold)    
        return overlapping_nodes
    
    def update_position(self, position, robot):
        robot = robot if robot else self.get_nearest_object(position)
        if robot:
            try:
                previous_position = self.tracked_objects[robot]
                same_position = np.array_equal(position, previous_position)
                if not same_position:
                    self.tracked_objects[robot] = position
                    print(f"updated: {robot} position")
            except:
                self.tracked_objects[robot] = position
                print(f"set the position of {robot}")


    def get_nearest_object(self, points, threshold=400):
        closest_id = None
        min_distance = gr.INF
        center_position = uf.find_center_of_rectangle(points)

        for obj_id, last_positions in self.tracked_objects.items():
            last_position_center = uf.find_center_of_rectangle(last_positions)
            distance = (uf.euclidean_distance(center_position, last_position_center))
            if distance < min_distance and distance < threshold:
                min_distance = distance
                closest_id = obj_id

        return closest_id

if __name__ == "__main__":
    main()