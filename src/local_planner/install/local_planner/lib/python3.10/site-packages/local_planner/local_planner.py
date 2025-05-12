import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

from nav_msgs.msg import Path
from nav_2d_msgs.msg import Path2D, Pose2DStamped, Twist2D
from geometry_msgs.msg import Pose2D
from dwb_msgs.srv import GetCriticScore
from dwb_msgs.msg import Trajectory2D
from builtin_interfaces.msg import Duration
from tf_transformations import euler_from_quaternion

from math import hypot

class LocalDWBPathGenerator(Node):
    def __init__(self):
        super().__init__('local_dwb_path_generator')

        self.declare_parameter('critic_name', 'PathAlign')

        self.global_path_sub = self.create_subscription(Path, '/global_plan', self.global_path_callback, 10)
        self.local_path_pub = self.create_publisher(Path, '/local_plan', 10)

        # ✅ 오직 한 개의 클라이언트만 생성
        self.client = self.create_client(GetCriticScore, '/controller_server/get_critic_score')

        # ✅ 타이머를 이용해 서비스 준비 여부를 나중에 체크
        self.service_check_timer = self.create_timer(1.0, self.check_service_ready)

        # 최초에는 서비스 준비되지 않았다고 가정
        self.service_ready = False

    def check_service_ready(self):
        if self.client.wait_for_service(timeout_sec=0.1):
            self.get_logger().info('✅ DWB get_critic_score service is available!')
            self.service_check_timer.cancel()
            self.service_ready = True
        else:
            self.get_logger().warn('⏳ Waiting for DWB get_critic_score service...')

    def global_path_callback(self, msg: Path):
        if not self.service_ready:
            self.get_logger().warn("🚫 DWB service not ready yet. Ignoring path callback.")
            return

        if len(msg.poses) < 2:
            self.get_logger().warn('Global path too short')
            return

        path2d = Path2D()
        path2d.header = msg.header

        for pose_stamped in msg.poses:
            pose = pose_stamped.pose
            q = pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

            p2d = Pose2D()
            p2d.x = pose.position.x
            p2d.y = pose.position.y
            p2d.theta = yaw
            path2d.poses.append(p2d)

        start_pose = Pose2DStamped()
        start_pose.header = msg.header
        start_pose.pose = path2d.poses[0]

        velocity = Twist2D()
        velocity.x = 0.1
        velocity.y = 0.0
        velocity.theta = 0.0

        traj = Trajectory2D()
        traj.velocity = velocity
        traj.time_offsets = [Duration(sec=0, nanosec=0)]
        traj.poses = path2d.poses

        request = GetCriticScore.Request()
        request.pose = start_pose
        request.velocity = velocity
        request.global_plan = path2d
        request.traj = traj
        request.critic_name = self.get_parameter('critic_name').get_parameter_value().string_value

        future = self.client.call_async(request)

        def response_callback(future):
            try:
                result = future.result()
                if result:
                    self.get_logger().info(f"[Critic Score] {result.score.raw_score:.3f}")
                else:
                    self.get_logger().warn("Service returned no result")
            except Exception as e:
                self.get_logger().error(f"Service call failed: {e}")

        future.add_done_callback(response_callback)

        # Trim local path based on distance (max 1.5m)
        local_path = Path()
        local_path.header = msg.header
        acc_dist = 0.0
        trimmed_poses = []
        last_pose = None

        for pose in msg.poses:
            if last_pose:
                dx = pose.pose.position.x - last_pose.pose.position.x
                dy = pose.pose.position.y - last_pose.pose.position.y
                acc_dist += hypot(dx, dy)
            trimmed_poses.append(pose)
            last_pose = pose
            if acc_dist > 1.5:
                break

        local_path.poses = trimmed_poses
        self.local_path_pub.publish(local_path)
        self.get_logger().info(f'📤 Published local plan with {len(local_path.poses)} poses')

def main(args=None):
    rclpy.init(args=args)
    node = LocalDWBPathGenerator()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
