from controller import Robot, Keyboard
import math
import asyncio
import websockets
import json
import threading
import random

class DroneController:
    def __init__(self):
        # Initialize the robot
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())
        
        # Command queue and current command storage
        self.command_queue = []
        self.current_command = None
        self.command_lock = threading.Lock()
        self.command_start_time = None
        self.command_duration = 500  # Her komut için varsayılan süre (ms)

        # Constants
        self.k_vertical_thrust = 0.0
        self.k_vertical_offset = 0.0
        self.k_vertical_p = 2.0  # Daha yumuşak yükseklik kontrolü
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0
        self.target_altitude = 0.0
        self.is_flying = False
        self.hover_thrust = 68.5  # Hover için gereken itme
        self.min_thrust = 10.0    # Minimum pervane hızı

        # Çarpma önleme sistemi için değişkenler
        self.collision_avoidance_enabled = True
        self.min_distance = 0.300  # 10cm minimum mesafe (metre cinsinden)
        self.distance_sensors = {}
        self.current_distances = {
            'on': 1000.0,    # ön
            'arka': 1000.0,  # arka
            'sol': 1000.0,   # sol
            'sag': 1000.0,   # sağ
            'yukari': 1000.0, # yukarı
            'asagi': 1000.0   # aşağı
        }

        # Initialize devices
        self.init_devices()
        self.motors = self.init_motors()
        print("Drone initialized and ready...")

        # Klavye kontrolü için eklenen kod
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # Klavye kontrol mesajlarını güncelle
        print("\nKlavye kontrolleri:")
        print("- 'SPACE': kalkış/iniş")
        print("- 'yukarı': ileri git")
        print("- 'aşağı': geri git")
        print("- 'sağ': sağa dön")
        print("- 'sol': sola dön")
        print("- 'shift + yukarı': hedef yüksekliği artır")
        print("- 'shift + aşağı': hedef yüksekliği azalt")
        print("- 'shift + sağ': sağa kay")
        print("- 'shift + sol': sola kay")

        self.last_space_press_time = 0  # SPACE tuşu için zaman takibi ekle
        self.space_debounce_time = 1.0  # 1 saniye bekleme süresi

        # WebSocket klavye kontrolü için yeni değişkenler
        self.websocket_roll = 0.0
        self.websocket_pitch = 0.0
        self.websocket_yaw = 0.0

        # Hız takibi için yeni değişkenler
        self.last_position = None
        self.last_time = None
        self.current_velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.current_speed = 0.0  # Toplam hız büyüklüğü
        
        # Motor hızları için değişkenler
        self.current_motor_speeds = {
            'front_left': 0.0,
            'front_right': 0.0,
            'rear_left': 0.0,
            'rear_right': 0.0
        }

    def init_devices(self):
        try:
            # Camera
            self.camera = self.robot.getDevice("camera")
            self.camera.enable(self.timestep)
            
            # LEDs
            self.front_left_led = self.robot.getDevice("front left led")
            self.front_right_led = self.robot.getDevice("front right led")
            
            # Sensors
            self.imu = self.robot.getDevice("inertial unit")
            self.imu.enable(self.timestep)
            
            self.gps = self.robot.getDevice("gps")
            self.gps.enable(self.timestep)
            
            self.compass = self.robot.getDevice("compass")
            self.compass.enable(self.timestep)
            
            self.gyro = self.robot.getDevice("gyro")
            self.gyro.enable(self.timestep)
            
            # Camera motors
            self.camera_roll = self.robot.getDevice("camera roll")
            self.camera_pitch = self.robot.getDevice("camera pitch")
            
            # Mesafe sensörleri GPS ve IMU tabanlı hesaplama ile simüle edilecek
            print("Mesafe sensörleri GPS ve IMU verilerinden hesaplanacak")
            
            print("All devices initialized successfully")
        except Exception as e:
            print(f"Error initializing devices: {str(e)}")
            raise

    def init_motors(self):
        try:
            motors = {
                'front_left': self.robot.getDevice("front left propeller"),
                'front_right': self.robot.getDevice("front right propeller"),
                'rear_left': self.robot.getDevice("rear left propeller"),
                'rear_right': self.robot.getDevice("rear right propeller")
            }
            
            for motor in motors.values():
                motor.setPosition(float('inf'))
                motor.setVelocity(2.0)  # Varsayılan hızı 2.0 olarak ayarla
            
            print("Motors initialized successfully with default speed 2.0")
            return motors
        except Exception as e:
            print(f"Error initializing motors: {str(e)}")
            raise

    def set_command(self, command, repeats, duration=None, speed_multiplier=1.0, target_height=None):
        """Komutu ve tekrar sayısını kuyruğa ekle"""
        with self.command_lock:
            self.command_queue.append((command, repeats, duration, speed_multiplier, target_height))
            print(f"Komut kuyruğa eklendi: {command}, Tekrar: {repeats}x" + 
                  (f", Süre: {duration}ms" if duration else "") +
                  f", Hız çarpanı: {speed_multiplier}" +
                  (f", Hedef yükseklik: {target_height}m" if target_height is not None else ""))

    def process_command(self):
        with self.command_lock:
            if self.current_command is None and self.command_queue:
                command_info = self.command_queue.pop(0)
                self.current_command = command_info[0]
                self.command_repeats = command_info[1]
                self.command_duration = command_info[2] if command_info[2] is not None else 1000
                self.current_speed_multiplier = command_info[3]
                
                # Hedef yüksekliği ayarla (eğer belirtilmişse)
                if len(command_info) > 4 and command_info[4] is not None:
                    self.target_altitude = command_info[4]
                    self.target_height = command_info[4]  # target_height'ı da güncelle
                
                self.command_start_time = self.robot.getTime() * 1000

                print(f"Yeni komut başlatılıyor: {self.current_command}, " +
                      f"Tekrar: {self.command_repeats}x, " +
                      f"Süre: {self.command_duration}ms, " +
                      f"Hız çarpanı: {self.current_speed_multiplier}, " +
                      f"Hedef yükseklik: {self.target_altitude}m")
            
            if self.current_command:
                elapsed_time = (self.robot.getTime() * 1000) - self.command_start_time
                if elapsed_time > self.command_duration:
                    self.command_repeats -= 1
                    if self.command_repeats <= 0:
                        print(f"Komut tamamlandı: {self.current_command}")
                        self.current_command = None
                        self.current_speed_multiplier = 1.0  # Hız çarpanını sıfırla
                    else:
                        self.command_start_time = self.robot.getTime() * 1000
                        print(f"Komut tekrarlanıyor: {self.current_command}, Kalan tekrar: {self.command_repeats}")
            
            command = self.current_command
            speed_multiplier = self.current_speed_multiplier if hasattr(self, 'current_speed_multiplier') else 1.0

        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        
        # Gerçek zamanlı çarpma önleme kontrolü
        collision_detected = False
        if self.collision_avoidance_enabled and self.is_flying and command:
            if command == "forward" and self.check_collision_risk('on', 0):
                print("⚠️ ÇARPMA RİSKİ: İleri hareket durduruldu!")
                collision_detected = True
            elif command == "backward" and self.check_collision_risk('arka', 0):
                print("⚠️ ÇARPMA RİSKİ: Geri hareket durduruldu!")
                collision_detected = True
            elif command == "strafe_left" and self.check_collision_risk('sol', 0):
                print("⚠️ ÇARPMA RİSKİ: Sol hareket durduruldu!")
                collision_detected = True
            elif command == "strafe_right" and self.check_collision_risk('sag', 0):
                print("⚠️ ÇARPMA RİSKİ: Sağ hareket durduruldu!")
                collision_detected = True
            elif command == "down" and self.check_collision_risk('asagi', 0):
                print("⚠️ ÇARPMA RİSKİ: Aşağı hareket durduruldu!")
                collision_detected = True
        
        if command == "takeoff":
            if not self.is_flying:
                self.is_flying = True
                self.k_vertical_thrust = self.hover_thrust
                self.k_vertical_offset = 0.6
                self.target_altitude = self.target_height
        elif command == "land":
            if self.is_flying:
                self.is_flying = False
                self.target_altitude = 0.0
        elif self.is_flying and not collision_detected:  # Çarpma riski yoksa hareket et
            if command == "forward":
                pitch_disturbance = -1.0 * speed_multiplier
            elif command == "backward":
                pitch_disturbance = 1.0 * speed_multiplier
            elif command == "strafe_right":
                roll_disturbance = -1.0 * speed_multiplier
            elif command == "strafe_left":
                roll_disturbance = 1.0 * speed_multiplier
            elif command == "right":
                yaw_disturbance = -0.78  # Hız çarpanı etkisiz
            elif command == "left":
                yaw_disturbance = 0.78  # Hız çarpanı etkisiz
            elif command == "up":
                if self.target_altitude < 4.0:
                    self.target_altitude += 0.0005  # Hız çarpanı etkisiz
            elif command == "down":
                if self.target_altitude > 0.2:
                    self.target_altitude -= 0.0009  # Hız çarpanı etkisiz
        elif collision_detected:
            # Çarpma riski varsa komutu iptal et ve kuyruğu temizle
            with self.command_lock:
                self.current_command = None
                self.command_queue.clear()
                print("🛑 Çarpma riski nedeniyle tüm komutlar iptal edildi!")
            
        return roll_disturbance, pitch_disturbance, yaw_disturbance

    def process_keyboard(self):
        """Klavye girişlerini işle ve kontrol değerlerini döndür"""
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        
        key = self.keyboard.getKey()
        while key > 0:
            current_time = self.robot.getTime()
            if key == ord(' '):  # SPACE tuşu
                if current_time - self.last_space_press_time > self.space_debounce_time:
                    if not self.is_flying:
                        self.is_flying = True
                        self.k_vertical_thrust = self.hover_thrust
                        self.k_vertical_offset = 0.6
                        # Varsayılan yükseklik yerine mevcut hedef yüksekliği kullan
                        self.target_altitude = getattr(self, 'target_height', 0.8)
                        print("Kalkış yapılıyor...")
                    else:
                        self.is_flying = False
                        self.k_vertical_thrust = 0.0
                        self.target_altitude = 0.0
                        print("İniş yapılıyor...")
                    self.last_space_press_time = current_time
            elif self.is_flying:  # Sadece uçarken diğer kontrolleri işle
                if key == Keyboard.UP:
                    pitch_disturbance = -2.5
                elif key == Keyboard.DOWN:
                    pitch_disturbance = 2.5
                elif key == Keyboard.RIGHT:
                    yaw_disturbance = -1.5
                elif key == Keyboard.LEFT:
                    yaw_disturbance = 1.5
                elif key == Keyboard.SHIFT + Keyboard.RIGHT:
                    roll_disturbance = -1.5
                elif key == Keyboard.SHIFT + Keyboard.LEFT:
                    roll_disturbance = 1.5
                elif key == Keyboard.SHIFT + Keyboard.UP:
                    self.target_altitude += 0.02
                    print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
                elif key == Keyboard.SHIFT + Keyboard.DOWN:
                    self.target_altitude -= 0.02
                    print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
            key = self.keyboard.getKey()
            
        return roll_disturbance, pitch_disturbance, yaw_disturbance

    def process_websocket_key(self, key):
        """WebSocket üzerinden gelen klavye girişlerini işle"""
        roll_disturbance = 0.0
        pitch_disturbance = 0.0
        yaw_disturbance = 0.0
        
        current_time = self.robot.getTime()
        
        if key == 'Space':  # SPACE tuşu
            if current_time - self.last_space_press_time > self.space_debounce_time:
                if not self.is_flying:
                    self.is_flying = True
                    self.k_vertical_thrust = self.hover_thrust
                    self.k_vertical_offset = 0.6
                    # Varsayılan yükseklik yerine mevcut hedef yüksekliği kullan
                    self.target_altitude = getattr(self, 'target_height', 0.8)
                    print("Kalkış yapılıyor...")
                else:
                    self.is_flying = False
                    self.k_vertical_thrust = 0.0
                    self.target_altitude = 0.0
                    print("İniş yapılıyor...")
                self.last_space_press_time = current_time
        elif self.is_flying:  # Sadece uçarken diğer kontrolleri işle
            if key == 'ArrowUp':
                pitch_disturbance = -2.5
            elif key == 'ArrowDown':
                pitch_disturbance = 2.5
            elif key == 'ArrowRight':
                yaw_disturbance = -1.5
            elif key == 'ArrowLeft':
                yaw_disturbance = 1.5
            elif key == 'ShiftRight':
                roll_disturbance = -1.5
            elif key == 'ShiftLeft':
                roll_disturbance = 1.5
            elif key == 'KeyW':
                self.target_altitude += 0.02
                print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
            elif key == 'KeyS':
                self.target_altitude -= 0.02
                print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
        
        return roll_disturbance, pitch_disturbance, yaw_disturbance

    def update_velocity(self):
        """GPS verilerinden hız hesaplama"""
        current_pos = self.gps.getValues()
        current_time = self.robot.getTime()
        
        if self.last_position is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                # Her eksendeki hız bileşenlerini hesapla (m/s)
                self.current_velocity = {
                    'x': (current_pos[0] - self.last_position[0]) / dt,
                    'y': (current_pos[1] - self.last_position[1]) / dt,
                    'z': (current_pos[2] - self.last_position[2]) / dt
                }
                
                # Toplam hız büyüklüğünü hesapla
                self.current_speed = math.sqrt(
                    self.current_velocity['x']**2 + 
                    self.current_velocity['y']**2 + 
                    self.current_velocity['z']**2
                )
        
        self.last_position = current_pos
        self.last_time = current_time

    def run_step(self):
        if self.robot.step(self.timestep) == -1:
            return False

        # Mesafe sensörlerini güncelle
        self.read_distance_sensors()
        
        # Klavye kontrollerini işle
        keyboard_roll, keyboard_pitch, keyboard_yaw = self.process_keyboard()
        
        # Mevcut komut işleme
        command_roll, command_pitch, command_yaw = self.process_command()
        
        # Klavye, komut ve websocket girişlerini birleştir
        roll_disturbance = keyboard_roll + command_roll + self.websocket_roll
        pitch_disturbance = keyboard_pitch + command_pitch + self.websocket_pitch
        yaw_disturbance = keyboard_yaw + command_yaw + self.websocket_yaw
        
        # Get sensor data
        roll = self.imu.getRollPitchYaw()[0]
        pitch = self.imu.getRollPitchYaw()[1]
        altitude = self.gps.getValues()[2]
        roll_acceleration = self.gyro.getValues()[0]
        pitch_acceleration = self.gyro.getValues()[1]
        
        # Update camera stabilization with clamped values
        camera_roll_position = self.clamp(-0.115 * roll_acceleration, -0.5, 0.5)
        camera_pitch_position = self.clamp(-0.1 * pitch_acceleration, -0.5, 0.5)
        self.camera_roll.setPosition(camera_roll_position)
        self.camera_pitch.setPosition(camera_pitch_position)
        
        # Calculate inputs
        roll_input = self.k_roll_p * self.clamp(roll, -1.0, 1.0) + roll_acceleration + roll_disturbance
        pitch_input = self.k_pitch_p * self.clamp(pitch, -1.0, 1.0) + pitch_acceleration + pitch_disturbance
        yaw_input = yaw_disturbance
        
        # Yumuşak yükseklik kontrolü
        clamped_difference_altitude = self.clamp(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
        vertical_input = self.k_vertical_p * math.pow(clamped_difference_altitude, 3.0)
        
        # Motor gücü hesaplama
        thrust = self.k_vertical_thrust if self.is_flying else self.min_thrust
        
        # Hover durumunda ön ve sağ motorlara ekstra güç ver
        is_hovering = self.is_flying and self.current_command != "takeoff" and self.current_command != "land"
        front_motor_boost = 0.3 if is_hovering else 0.0
        right_motor_boost = 0.12 if is_hovering else 0.0
        
        # Calculate motor inputs with minimum thrust and motor boosts
        front_left_motor_input = max(thrust + vertical_input - roll_input + pitch_input - yaw_input + front_motor_boost, self.min_thrust)
        front_right_motor_input = max(thrust + vertical_input + roll_input + pitch_input + yaw_input + front_motor_boost + right_motor_boost, self.min_thrust)
        rear_left_motor_input = max(thrust + vertical_input - roll_input - pitch_input + yaw_input, self.min_thrust)
        rear_right_motor_input = max(thrust + vertical_input + roll_input - pitch_input - yaw_input + right_motor_boost, self.min_thrust)
        
        # Update motor velocities
        self.motors['front_left'].setVelocity(front_left_motor_input)
        self.motors['front_right'].setVelocity(-front_right_motor_input)
        self.motors['rear_left'].setVelocity(-rear_left_motor_input)
        self.motors['rear_right'].setVelocity(rear_right_motor_input)
        
        # Motor hızlarını kaydet
        self.current_motor_speeds = {
            'front_left': front_left_motor_input,
            'front_right': front_right_motor_input,
            'rear_left': rear_left_motor_input,
            'rear_right': rear_right_motor_input
        }
        
        # Hız hesaplamalarını güncelle
        self.update_velocity()
        
        return True

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def parse_and_execute_code(self, code_text):
        try:
            print(f'Parsing code:\n{code_text}')
            
            # Güvenli bir şekilde locals() sözlüğü oluştur
            local_vars = {}
            
            # MockDrone sınıfını buraya taşı
            class MockDrone:
                def __init__(self, controller):
                    self.controller = controller
                    self.speed = 2
                    print('MockDrone initialized')

                def takeoff(self, height_cm=None):
                    print(f'MockDrone takeoff to {height_cm}cm' if height_cm else 'MockDrone takeoff')
                    target_height = 0.8
                    if height_cm is not None:
                        target_height = max(0.2, min(4.0, height_cm / 100.0))
                    self.controller.set_command('takeoff', 1, 3000, target_height=target_height)
                    self.controller.set_command('wait', 1, 3000)

                def land(self):
                    print('MockDrone land')
                    self.controller.set_command('land', 1)
                    self.controller.set_command('wait', 1, 3000)

                def fly(self, direction, distance_cm):
                    print(f'MockDrone fly: direction={direction}, distance={distance_cm}cm')
                    direction = direction.lower().strip()
                    
                    # Her 10cm için 1000ms temel süre
                    duration = max(500, int((distance_cm / 10) * 1000))
                    
                    direction_mapping = {
                        'ileri': 'forward',
                        'geri': 'backward',
                        'sol': 'strafe_left',
                        'sag': 'strafe_right',
                        'yukari': 'up',
                        'asagi': 'down'
                    }
                    
                    cmd = direction_mapping.get(direction)
                    if cmd:
                        self.controller.set_command(cmd, 1, duration)
                        self.controller.set_command('wait', 1, int(duration * 0.2))

                def turn(self, degrees):
                    print(f'MockDrone turn: {degrees} degrees')
                    duration = abs(degrees) * 20
                    
                    if degrees > 0:
                        self.controller.set_command('right', 1, duration=duration)
                    else:
                        self.controller.set_command('left', 1, duration=duration)
                    
                    self.controller.set_command('wait', 1, 3000)

                # Yeni hareket komutları
                def move_up(self, distance_cm):
                    print(f'MockDrone move_up: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('up', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def move_down(self, distance_cm):
                    print(f'MockDrone move_down: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('down', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def move_forward(self, distance_cm):
                    print(f'MockDrone move_forward: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('forward', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def move_backward(self, distance_cm):
                    print(f'MockDrone move_backward: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('backward', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def move_left(self, distance_cm):
                    print(f'MockDrone move_left: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('strafe_left', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def move_right(self, distance_cm):
                    print(f'MockDrone move_right: {distance_cm}cm')
                    duration = max(500, int((distance_cm / 10) * 1000))
                    self.controller.set_command('strafe_right', 1, duration)
                    self.controller.set_command('wait', 1, int(duration * 0.2))

                def turn_left(self, degrees):
                    print(f'MockDrone turn_left: {degrees} degrees')
                    duration = abs(degrees) * 20
                    self.controller.set_command('left', 1, duration=duration)
                    self.controller.set_command('wait', 1, 1000)

                def turn_right(self, degrees):
                    print(f'MockDrone turn_right: {degrees} degrees')
                    duration = abs(degrees) * 20
                    self.controller.set_command('right', 1, duration=duration)
                    self.controller.set_command('wait', 1, 1000)

                def read_sensor(self, direction):
                    """
                    Belirtilen yöndeki mesafe sensöründen veri oku
                    direction: 'on', 'arka', 'sol', 'sag', 'yukari', 'asagi'
                    Dönüş: Mesafe (santimetre cinsinden)
                    """
                    direction = direction.lower().strip()
                    
                    # Gerçek zamanlı sensör verilerini güncelle
                    self.controller.read_distance_sensors()
                    
                    # Mesafeyi döndür
                    distance = self.controller.current_distances.get(direction, 1000.0)
                    print(f'🔍 Sensor reading - {direction}: {distance:.1f}cm')
                    return distance
                
                def enable_collision_avoidance(self, enabled=True):
                    """Çarpma önleme sistemini aç/kapat"""
                    self.controller.collision_avoidance_enabled = enabled
                    print(f'Collision avoidance: {"enabled" if enabled else "disabled"}')
                
                def set_min_distance(self, distance_cm):
                    """Minimum güvenli mesafeyi ayarla"""
                    self.controller.min_distance = distance_cm / 100.0  # cm'yi metreye çevir
                    print(f'Minimum safe distance set to: {distance_cm}cm')

            # Mock time modülü oluştur
            class MockTime:
                def __init__(self, controller):
                    self.controller = controller
                
                def sleep(self, seconds):
                    print(f'⏱️ Sleep: {seconds} seconds')
                    duration_ms = int(seconds * 1000)
                    self.controller.set_command('wait', 1, duration_ms)
                    
                    # Gerçek zamanlı bekleme simülasyonu için
                    # Kısa beklemeler için sensör güncellemesi yap
                    if seconds <= 1.0:
                        self.controller.read_distance_sensors()

            # Thread fonksiyonlarını ayıkla ve sırayla çalıştır
            modified_code = ""
            if 'def thread_' in code_text:
                # Thread fonksiyonlarının içeriğini ayıkla
                thread_contents = []
                current_thread = []
                for line in code_text.split('\n'):
                    if line.strip().startswith('def thread_'):
                        if current_thread:
                            thread_contents.append('\n'.join(current_thread))
                            current_thread = []
                        continue
                    elif line.strip().startswith(('threads =', 't0 =', 't1 =', 'for t in', '#')):
                        continue
                    elif line.strip() and not line.strip().startswith('import'):
                        current_thread.append(line.strip())
                
                if current_thread:
                    thread_contents.append('\n'.join(current_thread))
                
                # Thread içeriklerini birleştir
                modified_code = '\n'.join(thread_contents)
            else:
                modified_code = code_text

            # Globals sözlüğünü güvenli şekilde oluştur
            safe_globals = {
                'print': print,
                'drone': MockDrone(self),
                'math': __import__('math'),
                'time': MockTime(self)
            }
            
            # Kodu çalıştır
            print('Executing modified code:\n', modified_code)
            exec(modified_code, safe_globals, local_vars)
            print('Code execution completed')
            
            return {'status': 'success', 'message': 'Kod başarıyla işlendi'}
            
        except Exception as e:
            print(f'Error executing code: {str(e)}')
            return {'status': 'error', 'message': f'Kod işlenirken hata: {str(e)}'}

    def get_drone_state(self):
        try:
            # GPS verilerini al
            gps_values = self.gps.getValues()
            position = {
                'x': round(gps_values[0], 2),
                'y': round(gps_values[1], 2),
                'z': round(gps_values[2], 2)
            }
            
            # IMU verilerini al (derece cinsinden)
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            orientation = {
                'roll': round(math.degrees(roll), 2),
                'pitch': round(math.degrees(pitch), 2),
                'yaw': round(math.degrees(yaw), 2)
            }
            
            # Pusula verilerini al
            compass_values = [round(v, 2) for v in self.compass.getValues()]
            
            # Gyro verilerini al (derece/saniye cinsinden)
            gyro_values = [round(math.degrees(v), 2) for v in self.gyro.getValues()]
            
            # Komut durumu
            command_status = {
                'current_command': self.current_command,
                'command_queue_length': len(self.command_queue),
                'is_executing': self.current_command is not None
            }
            
            # Hız bilgilerini ekle
            speed_data = {
                'velocity': {
                    'x': round(self.current_velocity['x'], 2),
                    'y': round(self.current_velocity['y'], 2),
                    'z': round(self.current_velocity['z'], 2)
                },
                'speed': round(self.current_speed, 2),  # Toplam hız (m/s)
                'motor_speeds': {
                    key: round(abs(value), 2) 
                    for key, value in self.current_motor_speeds.items()
                }
            }
            
            # Mesafe sensörü verileri
            distance_data = {
                'distances': {
                    direction: round(distance, 1) 
                    for direction, distance in self.current_distances.items()
                },
                'collision_avoidance_enabled': self.collision_avoidance_enabled,
                'min_distance_cm': round(self.min_distance * 100, 1)
            }
            
            # Genel durum
            return {
                'position': position,
                'orientation': orientation,
                'altitude': round(gps_values[2], 2),
                'compass': compass_values,
                'gyro': gyro_values,
                'is_flying': self.is_flying,
                'target_altitude': round(self.target_altitude, 2),
                'command_status': command_status,
                'speed_data': speed_data,  # Yeni hız verileri
                'distance_data': distance_data,  # Mesafe sensörü verileri
                'timestamp': self.robot.getTime()
            }
        except Exception as e:
            print(f'Error getting drone state: {str(e)}')
            return None

    def read_distance_sensors(self):
        """GPS, IMU ve kamera verilerini kullanarak mesafe hesaplama"""
        try:
            # GPS pozisyonunu al
            gps_values = self.gps.getValues()
            current_x, current_y, current_z = gps_values
            
            # IMU verilerini al (yönelim için)
            roll, pitch, yaw = self.imu.getRollPitchYaw()
            
            # Zemin yüksekliği (varsayılan olarak 0)
            ground_level = 0.0
            
            # Aşağı mesafesi - GPS z değerinden hesapla
            self.current_distances['asagi'] = max(5.0, (current_z - ground_level) * 100)  # cm cinsinden
            
            # Yukarı mesafesi - maksimum uçuş yüksekliğine göre
            max_altitude = 400.0  # 4 metre maksimum
            self.current_distances['yukari'] = max(10.0, (max_altitude - current_z) * 100)
            
            # Yatay mesafeler için basit fizik tabanlı hesaplama
            # Drone'un hızına ve yönelim açısına göre engel mesafesi tahmini
            
            # Hız vektörünü hesapla
            velocity_magnitude = self.current_speed
            
            # Yönelim açısına göre mesafe hesaplama
            # Eğer drone hareket ediyorsa, hareket yönündeki mesafeyi azalt
            base_distance = 150.0  # Temel mesafe (cm)
            
            # Hareket yönüne göre mesafe ayarlama - daha dinamik hesaplama
            # Temel mesafeleri pozisyona göre ayarla
            time_factor = self.robot.getTime() * 0.1  # Zaman faktörü ile dinamiklik
            
            # Pozisyon bazlı temel mesafeler
            base_distances = {
                'on': base_distance + math.sin(time_factor) * 20,
                'arka': base_distance + math.cos(time_factor) * 15,
                'sol': base_distance + math.sin(time_factor + 1) * 25,
                'sag': base_distance + math.cos(time_factor + 1) * 20,
            }
            
            # Hareket durumuna göre ayarlama
            if hasattr(self, 'current_command') and self.current_command:
                if self.current_command == 'forward':
                    # İleri giderken ön mesafeyi azalt
                    self.current_distances['on'] = max(15.0, base_distances['on'] - (velocity_magnitude * 60))
                    self.current_distances['arka'] = base_distances['arka'] + 50.0
                    self.current_distances['sol'] = base_distances['sol']
                    self.current_distances['sag'] = base_distances['sag']
                elif self.current_command == 'backward':
                    # Geri giderken arka mesafeyi azalt
                    self.current_distances['arka'] = max(15.0, base_distances['arka'] - (velocity_magnitude * 60))
                    self.current_distances['on'] = base_distances['on'] + 50.0
                    self.current_distances['sol'] = base_distances['sol']
                    self.current_distances['sag'] = base_distances['sag']
                elif self.current_command == 'strafe_left':
                    # Sola giderken sol mesafeyi azalt
                    self.current_distances['sol'] = max(15.0, base_distances['sol'] - (velocity_magnitude * 60))
                    self.current_distances['sag'] = base_distances['sag'] + 50.0
                    self.current_distances['on'] = base_distances['on']
                    self.current_distances['arka'] = base_distances['arka']
                elif self.current_command == 'strafe_right':
                    # Sağa giderken sağ mesafeyi azalt
                    self.current_distances['sag'] = max(15.0, base_distances['sag'] - (velocity_magnitude * 60))
                    self.current_distances['sol'] = base_distances['sol'] + 50.0
                    self.current_distances['on'] = base_distances['on']
                    self.current_distances['arka'] = base_distances['arka']
                else:
                    # Hover durumunda temel mesafeler
                    for direction, distance in base_distances.items():
                        self.current_distances[direction] = distance
            else:
                # Komut yoksa temel mesafeler
                for direction, distance in base_distances.items():
                    self.current_distances[direction] = distance
            
            # Pozisyon tabanlı engel simülasyonu
            # Belirli koordinatlarda sanal engeller oluştur
            obstacles = [
                {'x': 0.0, 'y': -2.5, 'radius': 1.0},  # Masa çevresinde engel
                {'x': -2.0, 'y': 0.0, 'radius': 0.8},  # Kutu çevresinde engel
                {'x': 2.0, 'y': 2.0, 'radius': 0.5},   # Sanal engel
                {'x': 1.0, 'y': 1.0, 'radius': 0.6},   # Ek sanal engel
                {'x': -1.0, 'y': -1.0, 'radius': 0.7}, # Ek sanal engel
            ]
            
            for obstacle in obstacles:
                # Drone ile engel arasındaki mesafe
                distance_to_obstacle = math.sqrt(
                    (current_x - obstacle['x'])**2 + 
                    (current_y - obstacle['y'])**2
                )
                
                if distance_to_obstacle < obstacle['radius'] + 1.0:  # 1 metre güvenlik mesafesi
                    # Engele yakın - hangi yönde olduğunu hesapla
                    angle_to_obstacle = math.atan2(
                        obstacle['y'] - current_y,
                        obstacle['x'] - current_x
                    )
                    
                    # Drone'un yönelim açısına göre engelin hangi tarafta olduğunu belirle
                    relative_angle = angle_to_obstacle - yaw
                    
                    # Açıyı -π ile π arasında normalize et
                    while relative_angle > math.pi:
                        relative_angle -= 2 * math.pi
                    while relative_angle < -math.pi:
                        relative_angle += 2 * math.pi
                    
                    # Mesafeyi cm cinsine çevir - daha dinamik hesaplama
                    base_obstacle_distance = (distance_to_obstacle - obstacle['radius']) * 100
                    # Zaman faktörü ile dinamik değişim
                    time_variation = math.sin(self.robot.getTime() * 0.5) * 10
                    obstacle_distance_cm = max(5.0, base_obstacle_distance + time_variation)
                    
                    # Açıya göre hangi sensörü etkileyeceğini belirle
                    if -math.pi/4 <= relative_angle < math.pi/4:
                        # Ön
                        self.current_distances['on'] = min(self.current_distances['on'], obstacle_distance_cm)
                        print(f"🚧 Engel tespit edildi - Ön: {obstacle_distance_cm:.1f}cm")
                    elif math.pi/4 <= relative_angle < 3*math.pi/4:
                        # Sol
                        self.current_distances['sol'] = min(self.current_distances['sol'], obstacle_distance_cm)
                        print(f"🚧 Engel tespit edildi - Sol: {obstacle_distance_cm:.1f}cm")
                    elif -3*math.pi/4 <= relative_angle < -math.pi/4:
                        # Sağ
                        self.current_distances['sag'] = min(self.current_distances['sag'], obstacle_distance_cm)
                        print(f"🚧 Engel tespit edildi - Sağ: {obstacle_distance_cm:.1f}cm")
                    else:
                        # Arka
                        self.current_distances['arka'] = min(self.current_distances['arka'], obstacle_distance_cm)
                        print(f"🚧 Engel tespit edildi - Arka: {obstacle_distance_cm:.1f}cm")
            
            # Minimum ve maksimum değerleri sınırla
            for direction in self.current_distances:
                self.current_distances[direction] = max(5.0, min(500.0, self.current_distances[direction]))
                
        except Exception as e:
            print(f"Mesafe sensörü okuma hatası: {e}")
            # Hata durumunda güvenli mesafeler ver
            for direction in self.current_distances:
                self.current_distances[direction] = 100.0

    def check_collision_risk(self, direction, distance_cm=0):
        """Çarpma riski kontrolü"""
        if not self.collision_avoidance_enabled:
            return False
        
        current_distance = self.current_distances.get(direction, 1000.0)
        min_distance_cm = self.min_distance * 100  # 50cm (güncellenmiş değer)
        
        # Gerçek zamanlı mesafe kontrolü
        return current_distance <= min_distance_cm

async def handle_websocket(websocket):
    global drone
    print('New client connected')
    try:
        async def send_state_updates():
            while True:
                try:
                    state = drone.get_drone_state()
                    if state:
                        # Komut kuyruğu boşsa ve aktif komut yoksa, işlemin bittiğini bildir
                        if state['command_status']['command_queue_length'] == 0 and not state['command_status']['current_command']:
                            await websocket.send(json.dumps({
                                'type': 'execution_complete',
                                'message': 'Tüm komutlar başarıyla tamamlandı'
                            }))
                        await websocket.send(json.dumps({
                            'type': 'state_update',
                            'data': state
                        }))
                    await asyncio.sleep(0.3)
                except Exception as e:
                    print(f'Durum güncellemesi gönderilirken hata: {e}')
                    break

        state_task = asyncio.create_task(send_state_updates())
        
        async for message in websocket:
            try:
                print(f'Alınan mesaj: {message}')  # Debug log
                data = json.loads(message)
                
                # Klavye kontrolü için yeni kontrol bloğu
                if 'type' in data and data['type'] == 'keyboard':
                    key = data.get('key')
                    # process_websocket_key metodunu çağır
                    roll, pitch, yaw = drone.process_websocket_key(key)
                    
                    # Değerleri doğrudan drone'un websocket değişkenlerine ata
                    drone.websocket_roll = roll
                    drone.websocket_pitch = pitch
                    drone.websocket_yaw = yaw
                    
                    # Tuş bırakıldığında değerleri sıfırla
                    if data.get('event') == 'keyup':
                        drone.websocket_roll = 0.0
                        drone.websocket_pitch = 0.0
                        drone.websocket_yaw = 0.0
                
                elif 'code' in data:
                    code = data['code'].replace('\\n', '\n')
                    print(f'İşlenen kod:\n{code}')
                    result = drone.parse_and_execute_code(code)
                    await websocket.send(json.dumps({
                        'type': 'code_response',
                        'status': result['status'],
                        'message': result['message']
                    }))
                else:
                    print(f'Geçersiz mesaj formatı: {data}')
                    await websocket.send(json.dumps({
                        'type': 'error',
                        'status': 'error',
                        'message': 'Geçersiz mesaj formatı'
                    }))
                    
            except json.JSONDecodeError as e:
                print(f'Geçersiz JSON: {message}')  # Debug log
                await websocket.send(json.dumps({
                    'type': 'error',
                    'status': 'error', 
                    'message': 'Geçersiz JSON formatı'
                }))
            except Exception as e:
                print(f'Mesaj işlenirken hata: {str(e)}')
                await websocket.send(json.dumps({
                    'type': 'error',
                    'status': 'error', 
                    'message': str(e)
                }))
                
    except websockets.exceptions.ConnectionClosed:
        print('Client disconnected')
    finally:
        if 'state_task' in locals():
            state_task.cancel()

def run_drone(drone):
    """Drone simülasyonunu çalıştıran fonksiyon"""
    while True:
        if not drone.run_step():
            break
    print("Drone simulation ended")

async def main():
    try:
        global drone
        # Create the drone controller
        drone = DroneController()
        
        # Create and start the drone thread
        drone_thread = threading.Thread(target=run_drone, args=(drone,))
        drone_thread.daemon = True
        drone_thread.start()

        # WebSocket sunucusunu başlat (basitleştirilmiş versiyon)
        server = await websockets.serve(
            handle_websocket,
            "0.0.0.0",  # Tüm ağ arayüzlerinden bağlantı kabul et
            2345,
            ping_interval=None,
            ping_timeout=None
        )
        print("WebSocket server running on ws://0.0.0.0:2345")
        await server.wait_closed()
            
    except Exception as e:
        print(f"Error in main: {str(e)}")
        raise

if __name__ == "__main__":
    drone = None  # Global drone değişkeni
    asyncio.run(main())