#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import select
import tty
import termios
import threading

class PPOTeleopNode(Node):
    def __init__(self):
        super().__init__('ppo_teleop_node')
        
        # Публикация команд скорости
        self.cmd_pub = self.create_publisher(Twist, '/loco_cmds', 10)
        
        # Параметры скорости
        self.linear_speed = 0.5   # м/с
        self.angular_speed = 1.0  # рад/с
        
        # Режимы скорости
        self.speed_modes = {
            '1': {'name': 'SLOW', 'linear': 0.2, 'angular': 0.5},
            '2': {'name': 'MEDIUM', 'linear': 0.5, 'angular': 1.0},
            '3': {'name': 'FAST', 'linear': 0.8, 'angular': 1.5}
        }
        
        # Состояние
        self.running = True
        self.current_twist = Twist()
        
        # Запуск
        self.select_speed_mode()
        self.print_controls()
        
        # Запуск потока для клавиатуры
        self.keyboard_thread = threading.Thread(target=self.keyboard_listener, daemon=True)
        self.keyboard_thread.start()
        
        # Таймер для публикации (20 Hz)
        self.timer = self.create_timer(0.05, self.publish_velocity)
        
        self.get_logger().info('Teleop node initialized. Press SPACE to stop, ESC to exit.')
    
    def select_speed_mode(self):
        """Выбор режима скорости"""
        print("\n" + "="*50)
        print("SELECT SPEED MODE:")
        print("="*50)
        print("1 - SLOW   (linear: 0.2 m/s, angular: 0.5 rad/s)")
        print("2 - MEDIUM (linear: 0.5 m/s, angular: 1.0 rad/s)")
        print("3 - FAST   (linear: 0.8 m/s, angular: 1.5 rad/s)")
        print("="*50)
        
        # Выводим приглашение один раз
        print("Press 1, 2, or 3: ", end='', flush=True)
        
        # Сохраняем текущие настройки терминала
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while True:
                # Ждем ввода с таймаутом
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key in self.speed_modes:
                        mode = self.speed_modes[key]
                        self.linear_speed = mode['linear']
                        self.angular_speed = mode['angular']
                        print(f"\n✓ Selected mode: {mode['name']}")
                        print(f"  Linear: {self.linear_speed} m/s")
                        print(f"  Angular: {self.angular_speed} rad/s")
                        break
                    elif key == '\x1b':  # ESC
                        print("\nExiting...")
                        self.running = False
                        sys.exit(0)
                    elif key in ['\n', '\r']:  # Игнорируем Enter
                        continue
                    else:
                        # Очищаем строку и выводим заново
                        print(f"\rInvalid key: {key}. Press 1, 2, or 3: ", end='', flush=True)
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    
    def print_controls(self):
        """Вывод управления"""
        print("\n" + "="*50)
        print("TELEOP CONTROLS:")
        print("="*50)
        print("Movement:")
        print("  W - Forward")
        print("  S - Backward")
        print("  Q - Strafe Left")
        print("  E - Strafe Right")
        print("\nRotation:")
        print("  A - Rotate Left")
        print("  D - Rotate Right")
        print("\nOther:")
        print("  SPACE - Emergency Stop")
        print("  ESC   - Exit")
        print("="*50)
        print("\nUse controls...\n")
    
    def keyboard_listener(self):
        """Отслеживание клавиш"""
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while self.running:
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    self.process_key(key.lower())
        finally:
            # ИСПРАВЛЕНИЕ: добавляем третий аргумент TCSADRAIN
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    
    def process_key(self, key):
        """Обработка нажатий"""
        # Сброс скоростей
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        # Движение вперед/назад
        if key == 'w':
            linear_x = self.linear_speed
        elif key == 's':
            linear_x = -self.linear_speed
        
        # Стрейф влево/вправо
        elif key == 'q':
            linear_y = self.linear_speed
        elif key == 'e':
            linear_y = -self.linear_speed
        
        # Поворот влево/вправо
        elif key == 'a':
            angular_z = self.angular_speed
        elif key == 'd':
            angular_z = -self.angular_speed
        
        # Экстренная остановка
        elif key == ' ':
            self.get_logger().warn('EMERGENCY STOP')
            # Все скорости 0
            pass
        
        # Выход
        elif key == '\x1b':  # ESC
            self.get_logger().info('Exiting teleop...')
            self.running = False
            self.current_twist = Twist()  # Остановка
            self.publish_velocity()
            sys.exit(0)
        
        else:
            # Игнорируем другие клавиши
            return
        
        # Обновляем команду
        self.current_twist.linear.x = linear_x
        self.current_twist.linear.y = linear_y
        self.current_twist.linear.z = 0.0
        self.current_twist.angular.x = 0.0
        self.current_twist.angular.y = 0.0
        self.current_twist.angular.z = angular_z
        
        # Логирование
        if linear_x != 0 or linear_y != 0 or angular_z != 0:
            self.get_logger().info(
                f'Cmd: lin=[{linear_x:.2f}, {linear_y:.2f}], ang={angular_z:.2f}',
                throttle_duration_sec=0.5
            )
        elif key == ' ':
            self.get_logger().info('Stopped')
    
    def publish_velocity(self):
        """Публикация текущей скорости"""
        self.cmd_pub.publish(self.current_twist)
    
    def __del__(self):
        """Деструктор - остановка робота при выходе"""
        self.current_twist = Twist()
        self.cmd_pub.publish(self.current_twist)
        self.get_logger().info('Teleop node destroyed, robot stopped')

def main(args=None):
    rclpy.init(args=args)
    node = PPOTeleopNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Остановка робота перед выходом
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()