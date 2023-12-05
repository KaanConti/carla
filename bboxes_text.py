import carla
import random
import queue
import numpy as np
import cv2
from PIL import Image


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
    
    
    
    
    
    
    
    
    
    
    
    
    
    # for bbox in bounding_boxes:
      #  points = [(int(bbox[i, 0]), int(bbox[i, 1])) for i in range(8)]
      
      
      
# #    '  bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
#         for bb in bounding_box_set:

#             test = bb.extent

#             print(test)
#             if bb.location.distance(vehicle.get_transform().location) < 100:
#                 draw_line(bb, (0,0,255, 255), "TrafficLight", K, world_2_camera)
                
#         bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficSigns)
#         for bb in bounding_box_set:
#             if bb.location.distance(vehicle.get_transform().location) < 100:
#                 test = bb.extent
#                 draw_line(bb, (255,0,255, 255), "TrafficSign", K, world_2_camera)
        
#         for npc in world.get_actors().filter('*vehicle*'):
#             if npc.id != vehicle.id:
#                 bb = npc.bounding_box
#                 # dist = npc.get_transform().location.distance(vehicle.get_transform().location '
                


def get_bounding_boxes(bbox_list, camera):
    """
    Creates 3D bounding boxes based on carla vehicle list and camera.
    """

    bounding_boxes = [get_bounding_box(bbox, camera) for bbox in bbox_list]
    # filter objects behind camera
    bounding_boxes = [bb for bb in bounding_boxes if all(bb[:, 2] > 0)]
    return bounding_boxes



def get_bounding_box(bbox_object, camera):
    """
    Returns 3D bounding box for a vehicle based on camera view.
    """

    bb_cords = _create_bb_points(bbox_object.extent)
    cords_x_y_z = _vehicle_to_sensor(bb_cords, bbox_object, camera)[:3, :]
    cords_y_minus_z_x = np.concatenate([cords_x_y_z[1, :], -cords_x_y_z[2, :], cords_x_y_z[0, :]])
    bbox = np.transpose(np.dot(camera.calibration, cords_y_minus_z_x))
    camera_bbox = np.concatenate([bbox[:, 0] / bbox[:, 2], bbox[:, 1] / bbox[:, 2], bbox[:, 2]], axis=1)
    return camera_bbox


def _create_bb_points(extent):
    """
    Returns 3D bounding box for a vehicle.
    """

    cords = np.zeros((8, 4))
    cords[0, :] = np.array([extent.x, extent.y, -extent.z, 1])
    cords[1, :] = np.array([-extent.x, extent.y, -extent.z, 1])
    cords[2, :] = np.array([-extent.x, -extent.y, -extent.z, 1])
    cords[3, :] = np.array([extent.x, -extent.y, -extent.z, 1])
    cords[4, :] = np.array([extent.x, extent.y, extent.z, 1])
    cords[5, :] = np.array([-extent.x, extent.y, extent.z, 1])
    cords[6, :] = np.array([-extent.x, -extent.y, extent.z, 1])
    cords[7, :] = np.array([extent.x, -extent.y, extent.z, 1])
    return cords


def _vehicle_to_sensor(cords, bbox, sensor):
    """
    Transforms coordinates of a vehicle bounding box to sensor.
    """

    world_cord = _vehicle_to_world(cords, bbox)
    
    
    
    sensor_cord = _world_to_sensor(world_cord, sensor)
    return sensor_cord

def _vehicle_to_world(cords, bbox_o):
    """
    Transforms coordinates of a vehicle bounding box to world.
    """



    bb_transform = carla.Transform(bbox_o.location)
    bb_vehicle_matrix = get_matrix(bb_transform)
    f = carla.Transform(bbox_o.location, bbox_o.rotation)
    vehicle_world_matrix = get_matrix(f)
    bb_world_matrix = np.dot(vehicle_world_matrix, bb_vehicle_matrix)
    world_cords = np.dot(bb_world_matrix, np.transpose(cords))
    return world_cords


def _world_to_sensor(cords, sensor):
    """
    Transforms world coordinates to sensor.
    """

    sensor_world_matrix = get_matrix(sensor.get_transform())
    world_sensor_matrix = np.linalg.inv(sensor_world_matrix)
    sensor_cords = np.dot(world_sensor_matrix, cords)
    return sensor_cords


def get_matrix(transform):
    """
    Creates matrix from carla transform.
    """

    rotation = transform.rotation
    location = transform.location
    c_y = np.cos(np.radians(rotation.yaw))
    s_y = np.sin(np.radians(rotation.yaw))
    c_r = np.cos(np.radians(rotation.roll))
    s_r = np.sin(np.radians(rotation.roll))
    c_p = np.cos(np.radians(rotation.pitch))
    s_p = np.sin(np.radians(rotation.pitch))
    matrix = np.matrix(np.identity(4))
    matrix[0, 3] = location.x
    matrix[1, 3] = location.y
    matrix[2, 3] = location.z
    matrix[0, 0] = c_p * c_y
    matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
    matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
    matrix[1, 0] = s_y * c_p
    matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
    matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
    matrix[2, 0] = s_p
    matrix[2, 1] = -c_p * s_r
    matrix[2, 2] = c_p * c_r
    return matrix







    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
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



def spawn_camera(world, bp, relative_location, relative_rotation, attach_to, fov, camera_id):
    camera_bp = bp_lib.find(bp)
    camera_bp.set_attribute('image_size_x', '1600')
    camera_bp.set_attribute('image_size_y', '900')
    camera_bp.set_attribute('fov', str(fov))

    camera_transform = carla.Transform(
        carla.Location(x=relative_location[0], y=relative_location[1], z=relative_location[2]),
        carla.Rotation(pitch=relative_rotation[0], yaw=relative_rotation[1], roll=relative_rotation[2])
    )

    camera = world.spawn_actor(camera_bp, camera_transform, attach_to=attach_to)

    # Attach a unique identifier to the camera
    camera.attributes['camera_id'] = camera_id

    return camera
                
                
                
                

client = carla.Client('localhost', 2000)
world = client.get_world()
bp_lib = world.get_blueprint_library()

spawn_points = world.get_map().get_spawn_points()
vehicle_bp = client.get_world().get_blueprint_library().filter('model3')[0]
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))


camera_relative_locations = [
    (1.5, 1.2, 1.4),   # front left
    (1.5, 0.0, 1.4),   # front middle
    (1.5, -1.2, 1.4),  # front right
    (-1.5, 1.2, 1.4),  # rear left
    (-1.5, 0.0, 1.4),  # rear middle
    (-1.5, -1.2, 1.4)  # rear right
]

camera_relative_rotations = [
    (0, 55, 0),    # front left
    (0, 0, 0),     # front middle
    (0, -55, 0),   # front right
    (0, 110, 0),   # rear left
    (0, 180, 0),   # rear middle
    (0, -110, 0)   # rear right
]

#same order as above
camera_fovs = [70,70,70,70,70,70]

# Unique identifiers for each camera
camera_ids = ['front_left', 'front_middle', 'front_right', 'rear_left', 'rear_middle', 'rear_right']


cameras = []
for i in range(len(camera_relative_locations)):
    camera = spawn_camera(world, 'sensor.camera.rgb', camera_relative_locations[i], camera_relative_rotations[i],
                          vehicle, camera_fovs[i], camera_ids[i])
    cameras.append(camera)



settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)

vehicle.set_autopilot(True)

image_queues = [queue.Queue() for _ in range(len(cameras))]
for i, camera in enumerate(cameras):
    camera.listen(image_queues[i].put)



edges = [[0, 1], [1, 3], [3, 2], [2, 0], [0, 4], [4, 5], [5, 1], [5, 7], [7, 6], [6, 4], [6, 2], [7, 3]]

for i in range(50):
    vehicle_bp = random.choice(bp_lib.filter('vehicle'))
    npc = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))
    if npc:
        npc.set_autopilot(True)
        
        
filter_patterns = ["vehicle.*", "traffic.traffic_light.*"]


def draw_line(bb, color, label, K, world_2_camera):


    verts = [v for v in bb.get_world_vertices(carla.Transform())]
    labeled = False
    for edge in edges:
        p1 = get_image_point(verts[edge[0]], K, world_2_camera)
        p2 = get_image_point(verts[edge[1]], K, world_2_camera)

        # Check if p1 and p2 are within the image boundaries
        if 0 <= p1[0] < image_w and 0 <= p1[1] < image_h and 0 <= p2[0] < image_w and 0 <= p2[1] < image_h:
            cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 1)

            if not labeled:
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                label_position = (int(p1[0]), int(p1[1]) + text_size[1])
                cv2.putText(img, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                labeled = True

def draw_line_vehicle(bb, actor, color, label, K, world_2_camera):
    verts = [v for v in bb.get_world_vertices(actor.get_transform())]
    labeled = False
    counter = 0
    for edge in edges:
        p1 = get_image_point(verts[edge[0]], K, world_2_camera)
        p2 = get_image_point(verts[edge[1]], K, world_2_camera)

        # Check if p1 and p2 are within the image boundaries
        if 0 <= p1[0] < image_w and 0 <= p1[1] < image_h and 0 <= p2[0] < image_w and 0 <= p2[1] < image_h:
            cv2.line(img, (int(p1[0]), int(p1[1])), (int(p2[0]), int(p2[1])), color, 1)
            counter = counter + 1

            # Draw label only once per loop iteration (counter == 2)
            if not labeled and counter == 2:
                text_size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0]
                label_position = (int(p1[0]), int(p1[1]) + text_size[1])
                cv2.putText(img, label, label_position, cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)
                labeled = True


image_w = 1600
image_h = 900
fov = 70
K = build_projection_matrix(image_w, image_h, fov) 



while True:
    world.tick()
    
    for i, queue in enumerate(image_queues):
        img = np.zeros((image_h, image_w, 4), dtype=np.uint8)  # Create a black image for each camera

        image = queue.get()
        img = np.reshape(np.copy(image.raw_data), (image.height, image.width, 4))

        camera_s = cameras[i]
        camera_transform = camera.get_transform()
        world_2_camera = np.array(camera_transform.get_inverse_matrix())
        
        
        all_bbox_list = []

        bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficLight)
        for bb in bounding_box_set:
            if bb.location.distance(vehicle.get_transform().location) < 100:
                all_bbox_list.append(bb)
                #draw_line(bb, (0,0,255, 255), "TrafficLight", K, world_2_camera)
                
        bounding_box_set = world.get_level_bbs(carla.CityObjectLabel.TrafficSigns)
        for bb in bounding_box_set:
            if bb.location.distance(vehicle.get_transform().location) < 100:
                all_bbox_list.append(bb)
                #draw_line(bb, (255,0,255, 255), "TrafficSign", K, world_2_camera)
        
        for npc in world.get_actors().filter('*vehicle*'):
            if npc.id != vehicle.id:
                bb = npc.bounding_box
                dist = npc.get_transform().location.distance(vehicle.get_transform().location)

                if dist < 100:
                    #draw_line_vehicle(bb, npc, (255,255,0, 255), "Vehicle", K, world_2_camera)
                    all_bbox_list.append(bb)
                    
                    
        
        wanted_bbox_list = []
        wanted_bbox_list = get_bounding_boxes(all_bbox_list, camera_s)
        
        print("\t\t\t\tLOOOOOOOOOOOOOOOOL\t\t\t")
        for bbox in wanted_bbox_list:
            draw_line(bb, (255,255,0,0), "None", K, world_2_camera)

        cv2.imshow('ImageWindowName',img)            
        cv2.imwrite(f"output_image_{i}.jpg", cv2.cvtColor(img, cv2.COLOR_RGBA2BGR))

    if cv2.waitKey(1) == ord('q'):
        break