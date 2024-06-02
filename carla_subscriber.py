import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import carla

class CarlaVehicleController(Node):
    def __init__(self):
        super().__init__('carla_vehicle_controller')
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(10.0)  # seconds

        # Load the 'Town02' map
        self.client.load_world('Town02')
        print("Loaded 'Town02' map.")

        self.world = self.client.get_world()
        self.blueprint_library = self.world.get_blueprint_library()
        self.vehicle = self.spawn_vehicle()
        self.spectator = self.world.get_spectator()
        self.pedestrian = self.spawn_pedestrian()
        
        self.subscription = self.create_subscription(
            Pose,
            'vehicle_position',
            self.update_vehicle_position,
            10)
        self.get_logger().info("Carla Vehicle Controller Node Started")

    def spawn_vehicle(self):
        car_bp = self.blueprint_library.filter('model3')[0]
        initial_spawn_point = carla.Transform(carla.Location(x=100, y=109, z=0.24), carla.Rotation(yaw=0))
        return self.world.spawn_actor(car_bp, initial_spawn_point)

    def update_vehicle_position(self, msg):
        new_location = carla.Location(x=msg.position.x, y=msg.position.y, z=msg.position.z)
        new_rotation = carla.Rotation(pitch=float(msg.orientation.x), yaw=float(msg.orientation.y), roll=float(msg.orientation.z))
        new_transform = carla.Transform(new_location, new_rotation)
        self.vehicle.set_transform(new_transform)
        self.update_spectator(new_transform)
        
        # Print the updated position and orientation
        print(f"Updated Vehicle Position: x={msg.position.x}, y={msg.position.y}, z={msg.position.z}")
        print(f"Updated Vehicle Orientation: pitch={msg.orientation.x}, yaw={msg.orientation.y}, roll={msg.orientation.z}")

    def spawn_pedestrian(self):
        pedestrian_bp = self.blueprint_library.filter("walker.pedestrian.*")[0]
        spawn_point = carla.Transform(carla.Location(x=180, y=109, z=0.24))
        print(f"Spawned pedestrian at x={spawn_point.location.x}, y={spawn_point.location.y}, z={spawn_point.location.z}")
        return self.world.spawn_actor(pedestrian_bp, spawn_point)

    def update_spectator(self, vehicle_transform):
        spectator_transform = carla.Transform(
            vehicle_transform.location + carla.Location(z=10) - vehicle_transform.rotation.get_forward_vector() * 10,
            carla.Rotation(pitch=-15, yaw=vehicle_transform.rotation.yaw))
        self.spectator.set_transform(spectator_transform)

def main(args=None):
    rclpy.init(args=args)
    carla_vehicle_controller = CarlaVehicleController()
    try:
        rclpy.spin(carla_vehicle_controller)
    except KeyboardInterrupt:
        pass  # Handle shutdown cleanly
    finally:
        carla_vehicle_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
