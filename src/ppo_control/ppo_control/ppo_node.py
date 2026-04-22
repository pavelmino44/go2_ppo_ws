import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from unitree_go.msg import LowState, LowCmd, MotorCmd
import onnxruntime as ort
import numpy as np
import struct
import transforms3d
import os
from ament_index_python.packages import get_package_share_directory

class Go2PolicyNode(Node):
    def __init__(self):
        super().__init__('ppo_node')

        try:
            package_share_dir = get_package_share_directory('ppo_control')
            model_path = os.path.join(package_share_dir, 'models', 'policy.onnx')
            self.get_logger().info(f'Loading model from: {model_path}')
            
            if not os.path.exists(model_path):
                raise FileNotFoundError(f'Model file not found: {model_path}')
            
            self.session = ort.InferenceSession(model_path)
            self.input_name = self.session.get_inputs()[0].name
            self.get_logger().info('Model loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise

        # Состояния
        self.joint_pos = np.zeros(12)
        self.joint_vel = np.zeros(12)
        self.base_lin_vel = np.zeros(3)
        self.base_ang_vel = np.zeros(3)
        self.projected_gravity = np.zeros(3)
        self.velocity_commands = np.zeros(3)
        self.actions = np.zeros(12)
        # self.height_scan = np.zeros(187)
        self.height_scan = np.ones(187, dtype=np.float32) * 0.5

        # Отладка
        self.running_mode = 'sim'
        self.check_counter = 0
        self.check_interval = 100

        # neutral позиции для Go2 (стоячее положение)
        self.neutral_positions = np.array([
            0.0, 0.8, -1.5,  # FR leg: hip, thigh, calf
            0.0, 0.8, -1.5,  # FL leg
            0.0, 0.8, -1.5,  # RR leg
            0.0, 0.8, -1.5   # RL leg
        ], dtype=np.float32)
        
        # Физические пределы суставов
        self.joint_limits_min = np.array([
            -0.5, 0.0, -2.5,  # FR
            -0.5, 0.0, -2.5,  # FL
            -0.5, 0.0, -2.5,  # RR
            -0.5, 0.0, -2.5   # RL
        ], dtype=np.float32)
        
        self.joint_limits_max = np.array([
            0.5, 2.0, -0.5,   # FR
            0.5, 2.0, -0.5,   # FL
            0.5, 2.0, -0.5,   # RR
            0.5, 2.0, -0.5    # RL
        ], dtype=np.float32)

        # Подписки
        self.create_subscription(LowState, '/lowstate', self.lowstate_cb, 10)
        self.create_subscription(PointCloud2, '/utlidar/cloud', self.lidar_cb, 10)
        self.create_subscription(Twist, '/loco_cmds', self.cmd_vel_cb, 10)

        # Публикация
        self.cmd_pub = self.create_publisher(LowCmd, '/lowcmd', 10)

        # Таймер
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz

    def lowstate_cb(self, msg: LowState):
        """
        Извлекает из LowState:
        - joint_pos (12,): относительные положения суставов
        - joint_vel (12,): относительные скорости суставов
        - base_lin_vel (3,): линейная скорость базы в системе тела
        - base_ang_vel (3,): угловая скорость базы в системе тела
        - projected_gravity (3,): проекция гравитации на оси тела
        """
        
        # 1. Положения суставов (12 значений)
        # В Go2 12 моторов: 4 ноги × 3 сустава (hip, thigh, calf)
        self.joint_pos = np.array([
            msg.motor_state[i].q for i in range(12)
        ], dtype=np.float32)
        
        # 2. Скорости суставов
        self.joint_vel = np.array([
            msg.motor_state[i].dq for i in range(12)
        ], dtype=np.float32)
        
        # 3. Угловая скорость из IMU (уже в системе тела)
        self.base_ang_vel = np.array([
            msg.imu_state.gyroscope[0],  # roll rate
            msg.imu_state.gyroscope[1],  # pitch rate
            msg.imu_state.gyroscope[2]   # yaw rate
        ], dtype=np.float32)
        
        # 4. Проекция гравитации из кватерниона IMU
        # Кватернион из мира в тело: [w, x, y, z]
        q = msg.imu_state.quaternion  # [w, x, y, z]
        
        # Преобразуем гравитационный вектор [0, 0, -1] в систему координат тела
        # Используя матрицу поворота из кватерниона
        R = transforms3d.quaternions.quat2mat([q[1], q[2], q[3], q[0]])  # [w, x, y, z] -> [x, y, z, w]
        
        # В мировой системе гравитация направлена вниз: g_world = [0, 0, -1]
        g_world = np.array([0, 0, -1], dtype=np.float32)
        
        # Проекция гравитации на оси тела: g_body = R.T @ g_world
        self.projected_gravity = (R.T @ g_world).astype(np.float32)
        
        # 5. Линейная скорость базы
        # В LowState нет прямой линейной скорости, но есть оценки из кинематики
        # Используем foot_force_est для определения фазы опоры
        # Для простоты можно интегрировать ускорения из IMU
        # Или использовать значения по умолчанию (0) если нет другого источника
        
        # Вариант 1: из интеграции ускорений IMU с компенсацией гравитации
        # accel_body = msg.imu_state.accelerometer - self.projected_gravity * 9.81
        # self.base_lin_vel += accel_body * self.dt
        
        # Вариант 2: из SportModeState (если доступен через другой топик)
        # Пока используем нули или последние известные значения
        if not hasattr(self, 'base_lin_vel'):
            self.base_lin_vel = np.zeros(3, dtype=np.float32)
        
        # Сохраняем временную метку для возможной интеграции
        self.last_imu_time = self.get_clock().now()
        
        # Логирование (опционально)
        self.get_logger().debug(
            f'LowState updated: joints=[{self.joint_pos[0]:.2f}, ...], '
            f'gyro=[{self.base_ang_vel[0]:.2f}, ...], '
            f'gravity=[{self.projected_gravity[0]:.2f}, ...]'
        )

    def lidar_cb(self, msg):
        # Преобразование PointCloud2 → height_scan
        self.height_scan = self.pc2_to_height_scan(msg)

    def cmd_vel_cb(self, msg):
        self.velocity_commands = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z
        ])

    def pc2_to_height_scan(self, cloud_msg):
        # Пример: разбиваем по углам или секторам
        # ...
        if self.running_mode == 'sim':
            height_scan = np.ones(187, dtype=np.float32) * 0.5  # 0.5м до пола
        
            # Небольшой шум для реалистичности
            height_scan += np.random.normal(0, 0.01, 187)
            
            # Клиппинг в ожидаемый диапазон
            height_scan = np.clip(height_scan, -1.0, 1.0)
            
            return height_scan
        else:
            return np.zeros(187)

    def check_observation(self):
        """Проверка observation на корректность"""
        self.get_logger().info('=== Observation Check ===')
        self.get_logger().info(f'base_lin_vel: {self.base_lin_vel}')
        self.get_logger().info(f'base_ang_vel: {self.base_ang_vel}')
        self.get_logger().info(f'projected_gravity: {self.projected_gravity}')
        self.get_logger().info(f'velocity_commands: {self.velocity_commands}')
        self.get_logger().info(f'joint_pos (first 3): {self.joint_pos[:3]}')
        self.get_logger().info(f'joint_vel (first 3): {self.joint_vel[:3]}')
        self.get_logger().info(f'actions (first 3): {self.actions[:3]}')
        self.get_logger().info(f'height_scan (first 10): {self.height_scan[:10]}')
        self.get_logger().info('========================')

    def control_loop(self):
        # Периодическая проверка observation
        self.check_counter += 1
        if self.check_counter >= self.check_interval:
            self.check_observation()
            self.check_counter = 0

        # Используем относительные позиции
        joint_pos_rel = self.joint_pos - self.neutral_positions
        
        obs = np.concatenate([
            self.base_lin_vel,
            self.base_ang_vel,
            self.projected_gravity,
            self.velocity_commands,
            joint_pos_rel,  # <-- относительные позиции
            self.joint_vel,
            self.actions,
            self.height_scan
        ]).astype(np.float32)

        # Прогон через политику
        actions = self.session.run(None, {self.input_name: obs.reshape(1, -1)})[0].flatten()
        
        # Жесткий клиппинг действий
        actions = np.clip(actions, -1.0, 1.0)
        
        # СРОЧНО: Уменьшаем масштаб действий в 20 раз!
        action_scale = 0.05  # Было 0.1, уменьшаем еще
        actions_scaled = actions * action_scale
        
        # Логирование для отладки
        self.get_logger().info(
            f'Actions raw: [{actions.min():.2f}, {actions.max():.2f}], '
            f'scaled: [{actions_scaled.min():.3f}, {actions_scaled.max():.3f}]',
            throttle_duration_sec=1.0
        )
    
        # Проверка на NaN/Inf
        if np.any(np.isnan(actions)) or np.any(np.isinf(actions)):
            self.get_logger().error('NaN or Inf in actions! Skipping...')
            return

        self.actions = actions.copy()

        # Формируем LowCmd
        cmd = LowCmd()
        for i in range(12):
            cmd.motor_cmd[i].mode = 0x01  # пример
            cmd.motor_cmd[i].q = float(actions[i])
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = Go2PolicyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()