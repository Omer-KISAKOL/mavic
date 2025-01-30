"""
Dron Kontrolcüsü:
- Gömülü sensörleri kullanarak dronu stabilize eder
- Roll/pitch/yaw stabilizasyonu için PID tekniği kullanır
- Dikey stabilizasyon için kübik fonksiyon kullanır
- Kamerayı stabilize eder
- Dronu bilgisayar klavyesi ile kontrol eder
"""

from controller import Robot, Keyboard
import math

# Yardımcı fonksiyonlar
def sign(x):
    if x > 0:
        return 1
    elif x < 0:
        return -1
    return 0

def clamp(value, low, high):
    return max(low, min(value, high))

# Ana robot sınıfı
class Mavic2Pro:
    def __init__(self):
        self.robot = Robot()
        self.timestep = int(self.robot.getBasicTimeStep())

        # Cihazları al ve etkinleştir
        self.camera = self.robot.getDevice("camera")
        self.camera.enable(self.timestep)
        
        self.front_left_led = self.robot.getDevice("front left led")
        self.front_right_led = self.robot.getDevice("front right led")
        
        self.imu = self.robot.getDevice("inertial unit")
        self.imu.enable(self.timestep)
        
        self.gps = self.robot.getDevice("gps")
        self.gps.enable(self.timestep)
        
        self.compass = self.robot.getDevice("compass")
        self.compass.enable(self.timestep)
        
        self.gyro = self.robot.getDevice("gyro")
        self.gyro.enable(self.timestep)
        
        self.keyboard = self.robot.getKeyboard()
        self.keyboard.enable(self.timestep)
        
        # Kamera motorları
        self.camera_roll = self.robot.getDevice("camera roll")
        self.camera_pitch = self.robot.getDevice("camera pitch")
        
        # Pervane motorları
        self.motors = []
        self.motors.append(self.robot.getDevice("front left propeller"))
        self.motors.append(self.robot.getDevice("front right propeller"))
        self.motors.append(self.robot.getDevice("rear left propeller"))
        self.motors.append(self.robot.getDevice("rear right propeller"))
        
        # Motorları sonsuz pozisyon moduna ayarla
        for motor in self.motors:
            motor.setPosition(float('inf'))
            motor.setVelocity(1.0)
        
        # Sabitler
        self.k_vertical_thrust = 68.5
        self.k_vertical_offset = 0.6
        self.k_vertical_p = 3.0
        self.k_roll_p = 50.0
        self.k_pitch_p = 30.0
        
        # Değişkenler
        self.target_altitude = 1.0
        
        # Başlangıç mesajlarını göster
        print("Dron başlatılıyor...\n")
        print("Dronu klavyenizle kontrol edebilirsiniz:")
        print("- 'yukarı': ileri git")
        print("- 'aşağı': geri git")
        print("- 'sağ': sağa dön")
        print("- 'sol': sola dön")
        print("- 'shift + yukarı': hedef yüksekliği artır")
        print("- 'shift + aşağı': hedef yüksekliği azalt")
        print("- 'shift + sağ': sağa kay")
        print("- 'shift + sol': sola kay")

    def run(self):
        while self.robot.step(self.timestep) != -1:
            time = self.robot.getTime()
            
            # Sensör verilerini al
            roll = self.imu.getRollPitchYaw()[0]
            pitch = self.imu.getRollPitchYaw()[1]
            altitude = self.gps.getValues()[2]
            roll_velocity = self.gyro.getValues()[0]
            pitch_velocity = self.gyro.getValues()[1]
            
            # LED'leri yanıp söndür
            led_state = int(time) % 2
            self.front_left_led.set(led_state)
            self.front_right_led.set(not led_state)
            
            # Kamerayı stabilize et
            self.camera_roll.setPosition(-0.115 * roll_velocity)
            self.camera_pitch.setPosition(-0.1 * pitch_velocity)
            
            # Klavye girişlerini işle
            roll_disturbance = pitch_disturbance = yaw_disturbance = 0.0
            key = self.keyboard.getKey()
            
            while key > 0:
                if key == Keyboard.UP:
                    pitch_disturbance = -2.0
                elif key == Keyboard.DOWN:
                    pitch_disturbance = 2.0
                elif key == Keyboard.RIGHT:
                    yaw_disturbance = -1.3
                elif key == Keyboard.LEFT:
                    yaw_disturbance = 1.3
                elif key == Keyboard.SHIFT + Keyboard.RIGHT:
                    roll_disturbance = -1.0
                elif key == Keyboard.SHIFT + Keyboard.LEFT:
                    roll_disturbance = 1.0
                elif key == Keyboard.SHIFT + Keyboard.UP:
                    self.target_altitude += 0.05
                    print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
                elif key == Keyboard.SHIFT + Keyboard.DOWN:
                    self.target_altitude -= 0.05
                    print(f"Hedef yükseklik: {self.target_altitude:.2f} [m]")
                key = self.keyboard.getKey()
            
            # Kontrol girdilerini hesapla
            roll_input = self.k_roll_p * clamp(roll, -1.0, 1.0) + roll_velocity + roll_disturbance
            pitch_input = self.k_pitch_p * clamp(pitch, -1.0, 1.0) + pitch_velocity + pitch_disturbance
            yaw_input = yaw_disturbance
            clamped_difference_altitude = clamp(self.target_altitude - altitude + self.k_vertical_offset, -1.0, 1.0)
            vertical_input = self.k_vertical_p * math.pow(clamped_difference_altitude, 3.0)
            
            # Motor hızlarını ayarla
            front_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input + pitch_input - yaw_input
            front_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input + pitch_input + yaw_input
            rear_left_motor_input = self.k_vertical_thrust + vertical_input - roll_input - pitch_input + yaw_input
            rear_right_motor_input = self.k_vertical_thrust + vertical_input + roll_input - pitch_input - yaw_input
            
            self.motors[0].setVelocity(front_left_motor_input)
            self.motors[1].setVelocity(-front_right_motor_input)
            self.motors[2].setVelocity(-rear_left_motor_input)
            self.motors[3].setVelocity(rear_right_motor_input)

# Ana program
controller = Mavic2Pro()
controller.run()
