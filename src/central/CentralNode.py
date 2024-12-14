import VideoToGraph as v2g
import time
import cv2 as cv
from util import UtilityFunctions as uf
from Graph import Graph as gr

def main():
    web_cam_close = "img/video/webcam_red_close.mov"
    web_cam_further_angle = "img/video/webcam_red_further_angle.mov"
    web_cam_further_top = "img/video/webcam_red_further_top.mov"
    robots = {
        'robot 1': 'R1:XX', #MAC address
        'robot 2': 'R2:XX:', 
    }
    video_fed = [web_cam_close, web_cam_further_angle, web_cam_further_top]
    for video_input in video_fed:
        driver_code(video_input, robots)
        print("Video feed completed: ", video_input)

def driver_code(video_input, robots):
    # parse the video adjust parameter to 0 to use webcam 
    central_node = CentralNode(video_input, robots)
    while len(central_node.vg.corners) < 4:
        print("Waiting for corners to be detected")
        time.sleep(1)

    central_node.vg.overlay_update_frame_interval = 1
    last_time = time.time()
    try:
        while True:
            if not central_node.vg.frame_queue.empty():
                frame = central_node.vg.frame_queue.get()
                frame = central_node.vg.overlay_text(frame, f"blocksize in cm: {central_node.vg.block_size_cm}", (50,50))
                frame = central_node.vg.overlay_text(frame, f"Update rate: {central_node.vg.overlay_update_frame_interval}", (100,100))
                cv.imshow(f'video feed: {video_input}', frame)
            if cv.waitKey(1) == ord('q') or central_node.vg.running == False:
                break
            if time.time() - last_time > 2:  
                last_time = time.time()
                instructions = central_node.run_solver()
                central_node.send_instructions(instructions)
        
            if cv.waitKey(1) == ord('r'):
                central_node.vg.block_size_cm = (central_node.vg.block_size_cm % 15) + 2

            if cv.waitKey(1) == ord('t'):
                central_node.vg.overlay_update_frame_interval = (central_node.vg.overlay_update_frame_interval % 20) + 2

    finally:
        central_node.tear_down()
        print("Final block finished")
    
class CentralNode:

    HEIGHT_CM = 75
    LENGTH_CM = 150
    def __init__(self, camera_input, robots):
        self.bluetooth_client = self.init_bluetooth_module()
        self.robots = self.init_robots(robots, self.bluetooth_client) # ensure connection is established
        self.vg = v2g.VideoToGraph(CentralNode.HEIGHT_CM, CentralNode.LENGTH_CM, camera_input)
        self.mrta_solver = self.init_mrta_solver()
        self.robot_calibration_and_sync()

    def init_robots(self, robots, bluetooth_client):
        print("initialized robots: ",robots)
        pass

    def init_bluetooth_module(self):
        pass

    def init_mrta_solver(self):
        pass

    def run_solver(self):
        # create and get the necessary input for mrta solver
        graph = self.vg.graph
        paths = self.vg.paths

        print("graph: ", graph)
        print("paths: ", paths)
        try:
            gr.print_path_weights(graph, paths['robot 1'])
        except Exception as e:
            print(e)

        task_stream = self.create_task_stream(graph, paths, self.robots)

        # run MRTA solver 
        solutions = 1 # self.mrta_solver.solve(graph, paths, task_stream)

        # convert solutions to instructions
        instructions = self.solutions_to_robot_instructions(solutions)
        return instructions

    def create_task_stream(self, weighted_graph, shortest_paths, robots):
        # create a task stream from the graph / path 
        pass

    def solutions_to_robot_instructions(self, solutions):
        return {'r1': 0, 'r2': 1}

    def send_instructions(self, instructions):
        for robot, instruction in instructions.items():
            self.send_instruction(robot, instruction)
        pass

    def send_instruction(self, robot, instruction):
        print(f"sent to robot: {robot}, instruction: {instruction}")
        return
        self.bluetooth_client.send(robot, instruction)

    def robot_calibration_and_sync(self):
        # ensure that movement is calibrated
        # move forward, orientation etc
        pass

    def tear_down(self):
        # Stop the thread and release resources 
        self.vg.tear_down()
        if self.vg.thread.is_alive():
            print(f"Thread {self.vg.thread.getName()} is alive: {self.vg.thread.is_alive()}")
            self.vg.thread.join()
        print("Tear down done")

if __name__ == "__main__":
    main()