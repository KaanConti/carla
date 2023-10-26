import carla
import random
import queue
import numpy as np
import cv2


class ActorBB:
    id_counters = {}  #Wörterbuch zur Verwaltung der Zähler für verschiedene type_ids

    def __init__(self, bb, type_id):
        self.bb = bb
        self.type_id = type_id
        self.unique_id = self.generate_unique_id(type_id)

    def generate_unique_id(self, type_id):
        if type_id not in ActorBB.id_counters:
            ActorBB.id_counters[type_id] = 0

        unique_id = ActorBB.id_counters[type_id]
        ActorBB.id_counters[type_id] += 1

        return unique_id
    
actor_bb_list = []   
    
    
def build_projection_matrix(w, h, fov):
    focal = w / (2.0 * np.tan(fov * np.pi / 360.0))
    K = np.identity(3)
    K[0, 0] = K[1, 1] = focal
    K[0, 2] = w / 2.0
    K[1, 2] = h / 2.0
    return K

def get_image_point(loc, K, w2c):
    point = np.array([loc.x, loc.y, loc.z, 1])
    point_camera = np.dot(w2c, point)
    point_camera = [point_camera[1], -point_camera[2], point_camera[0]]
    point_img = np.dot(K, point_camera)
    point_img[0] /= point_img[2]
    point_img[1] /= point_img[2]
    return point_img[0:2]


                
                

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

spawn_points = world.get_map().get_spawn_points()
vehicle_bp = bp_lib.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

camera_bp = bp_lib.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '1920')
camera_bp.set_attribute('image_size_y', '1208')
camera_bp.set_attribute('fov', '90')

camera_init_trans = carla.Transform(carla.Location(z=2))
camera = world.spawn_actor(camera_bp, camera_init_trans, attach_to=vehicle)
vehicle.set_autopilot(True)

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

image_w = camera_bp.get_attribute("image_size_x").as_int()
image_h = camera_bp.get_attribute("image_size_y").as_int()
fov = camera_bp.get_attribute("fov").as_float()
K = build_projection_matrix(image_w, image_h, fov)

image_queue = queue.Queue()
camera.listen(image_queue.put)

edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

for i in range(50):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    if npc:
        npc.set_autopilot(True)
        
        
filter_patterns = ["vehicle.*", "traffic.traffic_light.*"]



def draw_line(bb, color, label):
    forward_vec = vehicle.get_transform().get_forward_vector()
    ray = bb.location - vehicle.get_transform().location

    if forward_vec.dot(ray) > 1:
        verts = [v for v in bb.get_world_vertices(carla.Transform())]
        labeled = False
        for edge in edges:
            p1 = get_image_point(verts[edge[0]], K, world_2_camera)
            p2 = get_image_point(verts[edge[1]], K, world_2_camera)
            cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 1)
        
            if not labeled:
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                label_position = (int(p1[0]), int(p1[1]) + text_size[1])
                cv2.putText(img, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                labeled = True


def draw_line_vehicle(bb, actor, color, label):
    forward_vec = vehicle.get_transform().get_forward_vector()
    ray = actor.get_transform().location - vehicle.get_transform().location
    
    if forward_vec.dot(ray) > 1:
        print("hi")
        verts = [v for v in bb.get_world_vertices(actor.get_transform())]
        labeled = False
        counter = 0
        for edge in edges:
            p1 = get_image_point(verts[edge[0]], K, world_2_camera)
            p2 = get_image_point(verts[edge[1]], K, world_2_camera)
            
            if p2 is not None:
                cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 1)
                counter = counter +1
                if not labeled and counter == 2:
                    text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                    label_position = (int(p1[0]), int(p1[1]) + text_size[1])
                    cv2.putText(img, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                    labeled = True



while True:
    world.tick()
    image = image_queue.get()
    img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

    world_2_camera = np.array(camera.get_transform().get_inverse_matrix())
    k = carla.CityObjectLabel
    
    bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
    for bb in bounding_box_set:
        if bb.location.distance(vehicle.get_transform().location) < 100:
            draw_line(bb, (0,0,255, 255), "TrafficLight")
            
    bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficSigns)
    for bb in bounding_box_set:
        if bb.location.distance(vehicle.get_transform().location) < 100:
            draw_line(bb, (255,0,255, 255), "TrafficSign")
    
    for npc in world.get_actors().filter('*vehicle*'):
        if npc.id != vehicle.id:
            bb = npc.bounding_box
            dist = npc.get_transform().location.distance(vehicle.get_transform().location)

            if dist < 100:
                draw_line_vehicle(bb, npc, (255,255,0, 255), "Vehicle")

            

                    

    cv2.imshow('ImageWindowName',img)
    
    if cv2.waitKey(1) == ord('q'):
        break
