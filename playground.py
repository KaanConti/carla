import carla
import random
import queue
from PIL import Image
import os
import numpy as np
import time
import threading
import signal
import sys

# Ordner erstellen, um die Bilder zu speichern
output_folder = 'image_output'
os.makedirs(output_folder, exist_ok=True)

# Create a thread for the task

# Zählvariable für die Bilder
count = 0
count2 = 10000

class CarlaSimulation:
    def __init__(self):
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(5.0)
        self.used_v_spawn_point_list = []
        self.used_p_spawn_point_list = []
        self.camera_data = queue.Queue()
        
        try:
            self.world = self.client.get_world()
            self.world_map = self.world.get_map()
            print("Connected to CARLA")
        except Exception as e:
            print(f"Not connected to Carla: {e}")

    def SpawnVehicle(self, blueprint_name, spawn_point=None):
        vehicle_list = []
        
        was_none = False
        if spawn_point == None:
            was_none = True
            spawn_points = self.world.get_map().get_spawn_points()
        
        blueprint_library = self.world.get_blueprint_library()
        blueprint = random.choice(blueprint_library.filter(blueprint_name))
        
        if was_none:
            for i in range(50):
                spawn_point = random.choice(spawn_points)
                if spawn_point not in self.used_v_spawn_point_list:
                    vehicle = self.world.spawn_actor(blueprint, spawn_point)
                    vehicle_list.append(vehicle)
                    self.used_v_spawn_point_list.append(spawn_point)
            return vehicle_list
        
        else:
            blueprint.set_attribute("role_name", "hero")
            vehicle = self.world.spawn_actor(blueprint, spawn_point)
            return vehicle
        

    def SpawnPedestrian(self):
        pedestrian_list = []
        
        spawn_points = self.world.get_random_location()
        i = 0
        for i in range(50):
            pedestrian_bp = random.choice(self.world.get_blueprint_library().filter('walker.pedestrian.*'))
            spawn_point = random.choice(spawn_points)
            if spawn_points not in self.used_p_spawn_point_list:
                pedestrian = self.world.spawn_actor(pedestrian_bp, spawn_point)
                pedestrian_list.append(pedestrian)
                self.used_p_spawn_point_list.append(spawn_point)
            
        return pedestrian_list

    def SpawnEgoVehicleWithCamera(self, blueprint_name, spawn_point):
   
        ego_vehicle = self.SpawnVehicle("vehicle.mercedes.coupe", spawn_point)
    
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.semantic_segmentation') 
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1208')
        camera_bp.set_attribute('fov', '110')
        camera_bp.set_attribute("role_name", "fcsc")
      
        camera_transform = carla.Transform(carla.Location(x=1, y=1, z=2))  
        camera_sensor = self.world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
          
        camera_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '1920')
        camera_bp.set_attribute('image_size_y', '1208')
        camera_bp.set_attribute('fov', '110')
        camera_bp.set_attribute("role_name", "fcsc_rgb")
        
        camera_sensor1 = self.world.spawn_actor(camera_bp, camera_transform, attach_to=ego_vehicle)
    
        return ego_vehicle, [camera_sensor, camera_sensor1]

    def CenterCameraOnEgoVehicle(self, ego_vehicle=None):
        spectator = self.world.get_spectator()
        transform = self.ego_vehicle.get_transform()
        spectator.set_transform(carla.Transform(transform.location + carla.Location(z=15), carla.Rotation(pitch=-90)))
            
    def ProcessImageSem( k,raw_image):
        global count
        raw_image.save_to_disk(f"path/to/save/converted/image{count}", carla.libcarla.ColorConverter.CityScapesPalette)
       
        count = count + 1
        
    def ProcessImageRGB(k, raw_image):
        global count2
        raw_image.save_to_disk(f"path/to/save/converted/image{count2}", carla.libcarla.ColorConverter.Raw)
       
        count2 = count2 + 1
        
    def CenterViewCamera(self):
            spectator = self.world.get_spectator()
            transform = self.ego_vehicle.get_transform()
            spectator.set_transform(carla.Transform(transform.location + carla.Location(z=15), carla.Rotation(pitch=-90)))
            print("EWFGnGO")
            time.sleep(0.05)
        
       
    def signal_handler(self, sig, frame):
        print("Ctrl+C detected. Stopping the task...")
        self.exit_signal.set()
        self.task_thread.join()
        sys.exit(0)

        
    def RunSimulation(self):
        points = self.world.get_map().get_spawn_points()
        pos = random.choice(points)
        self.used_v_spawn_point_list.append(pos)
  
        
        # Spawnen von Fahrzeugen und Passanten
        self.vehicles = self.SpawnVehicle('vehicle.audi.tt')
        for vehicle in self.world.get_actors().filter('*vehicle*'):
            vehicle.set_autopilot(True)
        #self.pedestrians = self.SpawnPedestrian()
        blueprints = self.world.get_blueprint_library().filter('sensor.camera.*')
        for blueprint in blueprints:
            print(blueprint.id)

        camera_sensors = []
        # Spawnen des Ego-Fahrzeugs mit Kamera-Sensor
        self.ego_vehicle, camera_sensors = self.SpawnEgoVehicleWithCamera('vehicle.audi.tt', pos)
        self.ego_vehicle.set_autopilot(True)
    
    
        self.exit_signal = threading.Event()
        self.task_thread = threading.Thread(target=self.CenterViewCamera)
        self.task_thread.run()
        # Zentrieren der Kameraposition auf das Ego-Fahrzeug
        self.CenterCameraOnEgoVehicle(self.ego_vehicle)

        try:
            camera_sensors[0].listen(self.ProcessImageSem)
            camera_sensors[1].listen(self.ProcessImageRGB)

        
            print("Images are being saved in the 'output' directory.")
            while True:
                print(self.camera_data.qsize())
                
        except Exception as e:
            print(f"An error occurred: {e}")
            


        # Keep the main program running
        try:
            while True:
                pass
        except KeyboardInterrupt:
            pass
        finally:
            # Aufräumen und Simulation beenden
            print("Cleaning up...")
            for c in camera_sensors:
                c.destroy()
            self.ego_vehicle.destroy()
            self.task_thread.join()
          #  for vehicle in self.vehicles:
          #      vehicle.destroy()
          #  for pedestrian in self.pedestrians:
             #     pedestrian.destroy()

if __name__ == "__main__":
    carla_simulation = CarlaSimulation()
    carla_simulation.RunSimulation()









# Main loop to draw bounding boxes
while True:
    world.tick()
    image = image_queue.get()
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))
    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())

    i = 0
    
    
    for actor in world.get_actors().filter("traffic.traffic_light"):
        bb = actor.bounding_box
        dist = actor.get_transform().location.distance(vehicle.get_transform().location)
        print(f"Distanz zu Actor{i} beträgt {dist}")
        
        print(f"Actor ist ein {actor.type_id}")
        
        
        i = i+1
        if dist < 100:
            forward_vec = vehicle.get_transform().get_forward_vector()
            
            ray = actor.get_transform().location - vehicle.get_transform().location
            print(ray)
            if forward_vec.dot(ray) > 1:
                print("hi")
                verts = [v for v in bb.get_world_vertices(actor.get_transform())]
                for edge in edges:
                    p1 = get_image_point(verts[edge[0]], K, world_2_camera)
                    p2 = get_image_point(verts[edge[1]], K, world_2_camera)
                    
                    
                    if p2 is not None:
                        cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), (0,0,255, 255), 1)

                    else:
                        print("Wtf is going on")



    cv2.namedWindow('CARLA Bounding Boxes', cv2.WINDOW_AUTOSIZE)
    cv2.imshow('CARLA Bounding Boxes', img)

    if cv2.waitKey(1) == ord('q'):
        break

cv2.destroyAllWindows()