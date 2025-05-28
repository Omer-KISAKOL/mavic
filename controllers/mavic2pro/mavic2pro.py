from controller import Robot, Keyboard
import math
import asyncio
import websockets
import json
import threading

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
        elif self.is_flying:
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

            # Mock time modülü oluştur
            class MockTime:
                def __init__(self, controller):
                    self.controller = controller
                
                def sleep(self, seconds):
                    print(f'MockTime sleep: {seconds} seconds')
                    duration_ms = int(seconds * 1000)
                    self.controller.set_command('wait', 1, duration_ms)

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
                'timestamp': self.robot.getTime()
            }
        except Exception as e:
            print(f'Error getting drone state: {str(e)}')
            return None

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