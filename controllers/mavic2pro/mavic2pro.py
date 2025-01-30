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

    def set_command(self, command, repeats, duration=None, speed_multiplier=1.0):
        """Komutu ve tekrar sayısını kuyruğa ekle"""
        with self.command_lock:
            self.command_queue.append((command, repeats, duration, speed_multiplier))  # speed_multiplier eklendi
            print(f"Komut kuyruğa eklendi: {command}, Tekrar: {repeats}x" + 
                  (f", Süre: {duration}ms" if duration else "") +
                  f", Hız çarpanı: {speed_multiplier}")

    def process_command(self):
        with self.command_lock:
            # Eğer aktif komut yoksa ve kuyrukta komut varsa, yeni komut al
            if self.current_command is None and self.command_queue:
                command_info = self.command_queue.pop(0)  # Tuple olarak (komut, tekrar, süre, hız_çarpanı) al
                self.current_command = command_info[0]    # Komutu al
                self.command_repeats = command_info[1]    # Tekrar sayısını al
                self.command_duration = command_info[2] if command_info[2] is not None else 1000  # Süreyi al
                self.current_speed_multiplier = command_info[3]  # Hız çarpanını al
                self.command_start_time = self.robot.getTime() * 1000  # ms cinsinden
                
                print(f"Yeni komut başlatılıyor: {self.current_command}, Tekrar: {self.command_repeats}x, " +
                      f"Süre: {self.command_duration}ms, Hız çarpanı: {self.current_speed_multiplier}")
            
            # Eğer aktif komut varsa ve süresi dolduysa
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
                self.target_altitude = 0.8
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
                    self.target_altitude += 0.01  # Hız çarpanı etkisiz
            elif command == "down":
                if self.target_altitude > 0.2:
                    self.target_altitude -= 0.01  # Hız çarpanı etkisiz
            
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
                        self.target_altitude = 0.8
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
                    self.target_altitude = 0.8
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
        
        return True

    @staticmethod
    def clamp(value, min_value, max_value):
        return max(min(value, max_value), min_value)

    def parse_and_execute_code(self, code_text):
        """Frontend'den gelen Python kodunu çözümler ve komutlara dönüştürür"""
        try:
            print(f'Parsing code:\n{code_text}')  # Debug log
            
            # Güvenli bir şekilde locals() sözlüğü oluştur
            local_vars = {}
            
            # Sahte drone sınıfı oluştur
            class MockDrone:
                def __init__(self, controller):
                    self.controller = controller
                    self.speed = 2  # Varsayılan hız
                    # Birim dönüşüm sabitleri
                    self.CM_TO_STEPS = 0.02  # 1 cm için gereken adım sayısı
                    self.DEGREE_TO_MS = 20   # 1 derece dönüş için gereken milisaniye
                    print('MockDrone initialized')

                def set_speed(self, speed):
                    """
                    Drone'un hızını ayarlar.
                    Hız seviyeleri:
                    1: Yavaş hız
                    2: Normal hız (varsayılan)
                    3: Hızlı
                    4: Çok hızlı
                    """
                    speed = int(speed)  # Ondalıklı sayıları tam sayıya çevir
                    if speed < 1:
                        speed = 1
                    elif speed > 4:
                        speed = 4
                    print(f'MockDrone set_speed: {speed}')
                    self.speed = speed

                def takeoff(self, *args):  # *args ile herhangi bir parametre alabilir
                    """
                    Drone'u kaldırır. Parametre alsa bile görmezden gelir.
                    Sabit 3000ms süre ile kalkış yapar.
                    """
                    print('MockDrone takeoff')
                    self.controller.set_command('takeoff', 1, 3000)  # Sabit 3000ms kalkış süresi
                    self.controller.set_command('wait', 1, 3000)  # Kalkış sonrası 3 saniye bekleme
                
                def land(self):
                    print('MockDrone land')
                    self.controller.set_command('land', 1)
                    self.controller.set_command('wait', 1)
                
                def fly(self, direction, distance_cm):
                    """
                    distance_cm: Santimetre cinsinden mesafe
                    Yön seçenekleri:
                    - ileri: İleri hareket
                    - geri: Geri hareket
                    - sol: Sola kayma (strafe left)
                    - sag: Sağa kayma (strafe right)
                    - yukari: Yukarı hareket
                    - asagi: Aşağı hareket
                    """
                    print(f'MockDrone fly: direction={direction}, distance={distance_cm}cm, speed={self.speed}')
                    direction = direction.lower().strip()
                    # Santimetreden adım sayısına çevirme
                    steps = max(1, int(distance_cm * self.CM_TO_STEPS))
                    
                    # Sabit süre kullan ama motor hızını ayarla
                    duration = 3000  # Sabit 3 saniye
                    speed_multiplier = {1: 1.0, 2: 1.5, 3: 2.0}  # Motor hız çarpanları
                    
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
                        print(f'Converting {distance_cm}cm to {steps} steps for {cmd} with motor speed multiplier {speed_multiplier[self.speed]}')
                        # Hız çarpanını komutla birlikte gönder
                        self.controller.set_command(cmd, steps, duration, speed_multiplier=speed_multiplier[self.speed])
                        self.controller.set_command('wait', 1, 3500)  # Sabit bekleme süresi

                def turn(self, degrees):
                    """
                    Drone'u belirtilen derece kadar döndürür.
                    Hıza göre motor hızı değişir:
                    - Hız 1 (yavaş): Motor hızı x0.5
                    - Hız 2 (normal): Motor hızı x1.0
                    - Hız 3 (hızlı): Motor hızı x1.5
                    - Hız 4 (çok hızlı): Motor hızı x2.0
                    """
                    print(f'MockDrone turn: {degrees} degrees at speed {self.speed}')
                    
                    duration = abs(degrees) * 20  # Her derece için 20ms dönüş süresi
                    speed_multiplier = {1: 0.5, 2: 1.0, 3: 1.5, 4: 2.0}  # Motor hız çarpanları
                    
                    repeats = 1
                    if degrees > 0:
                        print(f'Turning right {degrees} degrees with motor speed multiplier {speed_multiplier[self.speed]}')
                        self.controller.set_command('right', repeats, duration=duration, speed_multiplier=speed_multiplier[self.speed])
                    else:
                        print(f'Turning left {abs(degrees)} degrees with motor speed multiplier {speed_multiplier[self.speed]}')
                        self.controller.set_command('left', repeats, duration=duration, speed_multiplier=speed_multiplier[self.speed])
                    
                    self.controller.set_command('wait', 1, 3000)  # Sabit bekleme süresi

                def fly_to(self, x_cm, y_cm, z_cm):
                    """
                    x_cm, y_cm, z_cm: Santimetre cinsinden koordinatlar
                    """
                    print(f'MockDrone fly_to: x={x_cm}cm, y={y_cm}cm, z={z_cm}cm')
                    # Santimetreden adım sayısına çevirme
                    x_steps = max(1, int(abs(x_cm) * self.CM_TO_STEPS))
                    z_steps = max(1, int(abs(z_cm) * self.CM_TO_STEPS))
                    
                    self.controller.set_command('forward', x_steps)
                    if z_cm > 0:
                        self.controller.set_command('up', z_steps)
                    else:
                        self.controller.set_command('down', z_steps)
                    self.controller.set_command('wait', 1)

                def return_to_start(self):
                    print('MockDrone return_to_start')
                    self.controller.set_command('land', 1)
                    self.controller.set_command('wait', 1)
                    self.controller.set_command('takeoff', 1)

                def stream_on(self):
                    print('MockDrone stream_on')
                    # Kamera stream simülasyonu

                def stream_off(self):
                    print('MockDrone stream_off')
                    # Kamera stream kapatma simülasyonu

                def emergency_stop(self):
                    print('MockDrone emergency_stop')
                    self.controller.set_command('land', 1)

                def stop(self):
                    print('MockDrone stop')
                    self.controller.set_command('wait', 1)

                # Sensör fonksiyonları
                def get_temperature(self):
                    return 25.0  # Simüle edilmiş sıcaklık değeri (Celsius)

                def get_humidity(self):
                    return 45.0  # Simüle edilmiş nem değeri (%)

                def get_pressure(self):
                    return 1013.25  # Simüle edilmiş basınç değeri (hPa)

                def get_height(self):
                    return 150.0  # Simüle edilmiş yükseklik değeri (cm)

                def get_acceleration(self, axis):
                    return 0.0  # Simüle edilmiş ivme değeri (m/s²)
            
            local_vars['drone'] = MockDrone(self)
            
            # Mock sleep fonksiyonu için controller referansını saklayalım
            controller = self
            
            def mock_sleep(seconds):
                print(f'Mock sleep: {seconds} seconds')
                controller.set_command('wait', int(seconds))
            
            # Mock time modülü oluştur
            local_vars['time'] = type('MockTime', (), {'sleep': staticmethod(mock_sleep)})()
            
            # Kodu çalıştır
            print('Executing code...')
            exec(code_text, {
                '__builtins__': {
                    'print': print,
                    'range': range,
                    'int': int,
                    'float': float,
                    'str': str,
                    'abs': abs
                }
            }, local_vars)
            print('Code execution completed')
            
            return {'status': 'success', 'message': 'Kod başarıyla işlendi'}
            
        except Exception as e:
            print(f'Error executing code: {str(e)}')  # Debug log
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
            8765,
            ping_interval=None,
            ping_timeout=None
        )
        print("WebSocket server running on ws://0.0.0.0:8765")
        await server.wait_closed()
            
    except Exception as e:
        print(f"Error in main: {str(e)}")
        raise

if __name__ == "__main__":
    drone = None  # Global drone değişkeni
    asyncio.run(main())