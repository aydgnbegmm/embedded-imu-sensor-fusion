from ultralytics import YOLO
import cv2
import numpy as np
from collections import deque
import time
import socket
import threading
import json
import logging
from dataclasses import dataclass
import torch
from enum import Enum, auto
import os
from datetime import datetime
from typing import Optional, Tuple, List, Dict, Any
import smbus
import math


class mpu6050:
    """MPU6050 sensör sınıfı - verilen koddan alındı"""
    # Global Variables
    GRAVITIY_MS2 = 9.80665
    address = None
    bus = None

    # Scale Modifiers
    ACCEL_SCALE_MODIFIER_2G = 16384.0
    ACCEL_SCALE_MODIFIER_4G = 8192.0
    ACCEL_SCALE_MODIFIER_8G = 4096.0
    ACCEL_SCALE_MODIFIER_16G = 2048.0

    GYRO_SCALE_MODIFIER_250DEG = 131.0
    GYRO_SCALE_MODIFIER_500DEG = 65.5
    GYRO_SCALE_MODIFIER_1000DEG = 32.8
    GYRO_SCALE_MODIFIER_2000DEG = 16.4

    # Pre-defined ranges
    ACCEL_RANGE_2G = 0x00
    ACCEL_RANGE_4G = 0x08
    ACCEL_RANGE_8G = 0x10
    ACCEL_RANGE_16G = 0x18

    GYRO_RANGE_250DEG = 0x00
    GYRO_RANGE_500DEG = 0x08
    GYRO_RANGE_1000DEG = 0x10
    GYRO_RANGE_2000DEG = 0x18

    FILTER_BW_256=0x00
    FILTER_BW_188=0x01
    FILTER_BW_98=0x02
    FILTER_BW_42=0x03
    FILTER_BW_20=0x04
    FILTER_BW_10=0x05
    FILTER_BW_5=0x06

    # MPU-6050 Registers
    PWR_MGMT_1 = 0x6B
    PWR_MGMT_2 = 0x6C

    ACCEL_XOUT0 = 0x3B
    ACCEL_YOUT0 = 0x3D
    ACCEL_ZOUT0 = 0x3F

    TEMP_OUT0 = 0x41

    GYRO_XOUT0 = 0x43
    GYRO_YOUT0 = 0x45
    GYRO_ZOUT0 = 0x47

    ACCEL_CONFIG = 0x1C
    GYRO_CONFIG = 0x1B
    MPU_CONFIG = 0x1A

    def __init__(self, address, bus=1):
        self.address = address
        self.bus = smbus.SMBus(bus)
        # Wake up the MPU-6050 since it starts in sleep mode
        self.bus.write_byte_data(self.address, self.PWR_MGMT_1, 0x00)

    def read_i2c_word(self, register):
        """Read two i2c registers and combine them."""
        high = self.bus.read_byte_data(self.address, register)
        low = self.bus.read_byte_data(self.address, register + 1)
        value = (high << 8) + low
        if (value >= 0x8000):
            return -((65535 - value) + 1)
        else:
            return value

    def get_temp(self):
        """Reads the temperature from the onboard temperature sensor of the MPU-6050."""
        raw_temp = self.read_i2c_word(self.TEMP_OUT0)
        actual_temp = (raw_temp / 340.0) + 36.53
        return actual_temp

    def set_accel_range(self, accel_range):
        """Sets the range of the accelerometer to range."""
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.ACCEL_CONFIG, accel_range)

    def read_accel_range(self, raw = False):
        """Reads the range the accelerometer is set to."""
        raw_data = self.bus.read_byte_data(self.address, self.ACCEL_CONFIG)
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.ACCEL_RANGE_2G:
                return 2
            elif raw_data == self.ACCEL_RANGE_4G:
                return 4
            elif raw_data == self.ACCEL_RANGE_8G:
                return 8
            elif raw_data == self.ACCEL_RANGE_16G:
                return 16
            else:
                return -1

    def get_accel_data(self, g = False):
        """Gets and returns the X, Y and Z values from the accelerometer."""
        x = self.read_i2c_word(self.ACCEL_XOUT0)
        y = self.read_i2c_word(self.ACCEL_YOUT0)
        z = self.read_i2c_word(self.ACCEL_ZOUT0)

        accel_scale_modifier = None
        accel_range = self.read_accel_range(True)

        if accel_range == self.ACCEL_RANGE_2G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G
        elif accel_range == self.ACCEL_RANGE_4G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_4G
        elif accel_range == self.ACCEL_RANGE_8G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_8G
        elif accel_range == self.ACCEL_RANGE_16G:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_16G
        else:
            accel_scale_modifier = self.ACCEL_SCALE_MODIFIER_2G

        x = x / accel_scale_modifier
        y = y / accel_scale_modifier
        z = z / accel_scale_modifier

        if g is True:
            return {'x': x, 'y': y, 'z': z}
        elif g is False:
            x = x * self.GRAVITIY_MS2
            y = y * self.GRAVITIY_MS2
            z = z * self.GRAVITIY_MS2
            return {'x': x, 'y': y, 'z': z}

    def set_gyro_range(self, gyro_range):
        """Sets the range of the gyroscope to range."""
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, 0x00)
        self.bus.write_byte_data(self.address, self.GYRO_CONFIG, gyro_range)

    def set_filter_range(self, filter_range=FILTER_BW_256):
        """Sets the low-pass bandpass filter frequency"""
        EXT_SYNC_SET = self.bus.read_byte_data(self.address, self.MPU_CONFIG) & 0b00111000
        return self.bus.write_byte_data(self.address, self.MPU_CONFIG,  EXT_SYNC_SET | filter_range)

    def read_gyro_range(self, raw = False):
        """Reads the range the gyroscope is set to."""
        raw_data = self.bus.read_byte_data(self.address, self.GYRO_CONFIG)
        if raw is True:
            return raw_data
        elif raw is False:
            if raw_data == self.GYRO_RANGE_250DEG:
                return 250
            elif raw_data == self.GYRO_RANGE_500DEG:
                return 500
            elif raw_data == self.GYRO_RANGE_1000DEG:
                return 1000
            elif raw_data == self.GYRO_RANGE_2000DEG:
                return 2000
            else:
                return -1

    def get_gyro_data(self):
        """Gets and returns the X, Y and Z values from the gyroscope."""
        x = self.read_i2c_word(self.GYRO_XOUT0)
        y = self.read_i2c_word(self.GYRO_YOUT0)
        z = self.read_i2c_word(self.GYRO_ZOUT0)

        gyro_scale_modifier = None
        gyro_range = self.read_gyro_range(True)

        if gyro_range == self.GYRO_RANGE_250DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG
        elif gyro_range == self.GYRO_RANGE_500DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_500DEG
        elif gyro_range == self.GYRO_RANGE_1000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_1000DEG
        elif gyro_range == self.GYRO_RANGE_2000DEG:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_2000DEG
        else:
            gyro_scale_modifier = self.GYRO_SCALE_MODIFIER_250DEG

        x = x / gyro_scale_modifier
        y = y / gyro_scale_modifier
        z = z / gyro_scale_modifier

        return {'x': x, 'y': y, 'z': z}

    def get_all_data(self):
        """Reads and returns all the available data."""
        temp = self.get_temp()
        accel = self.get_accel_data()
        gyro = self.get_gyro_data()
        return [accel, gyro, temp]


class OrientationFilter:
    """MPU6050 verileri ile yön hesaplama sınıfı"""
    def __init__(self, alpha=0.96):
        self.alpha = alpha  # Complementary filter katsayısı
        self.pitch = 0.0
        self.roll = 0.0
        self.yaw = 0.0
        self.last_time = time.time()
        
    def update(self, accel_data, gyro_data):
        """Accelerometer ve gyroscope verilerini kullanarak yön hesapla"""
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time
        
        # Accelerometer'dan açı hesapla
        accel_pitch = math.atan2(accel_data['y'], 
                                math.sqrt(accel_data['x']**2 + accel_data['z']**2)) * 180/math.pi
        accel_roll = math.atan2(-accel_data['x'], accel_data['z']) * 180/math.pi
        
        # Gyroscope entegrasyonu
        self.pitch += gyro_data['x'] * dt
        self.roll += gyro_data['y'] * dt
        self.yaw += gyro_data['z'] * dt
        
        # Complementary filter uygula
        self.pitch = self.alpha * self.pitch + (1 - self.alpha) * accel_pitch
        self.roll = self.alpha * self.roll + (1 - self.alpha) * accel_roll
        
        return {
            'pitch': self.pitch,
            'roll': self.roll, 
            'yaw': self.yaw
        }


class SistemDurumu(Enum):
    BASLATILIYOR = auto()
    OTONOM_MOD = auto()
    KIRMIZI_BALON_ARAMA = auto()
    HEDEF_KILITLEME = auto()
    BEKLEME = auto()
    KAPATILIYOR = auto()


@dataclass
class Config:
    CAMERA_ID: int = 0
    CAMERA_WIDTH: int = 640
    CAMERA_HEIGHT: int = 480
    MODEL_PATH: str = "b.onnx"
    MAX_MISSING_FRAMES: int = 15
    KALMAN_PROCESS_NOISE: float = 0.001
    KALMAN_MEASUREMENT_NOISE: float = 5.0
    MAX_VELOCITY_PPS: float = 200.0
    MAX_ACCELERATION_PPS2: float = 100.0
    SMOOTHING_ALPHA: float = 0.7
    MEASUREMENT_QUALITY_THRESHOLD: float = 0.5
    DEBUG_MODE: bool = True
    CONFIDENCE: float = 0.6
    TARGET_FPS: int = 30
    USE_CUDA: bool = True
    BEKLEME_SURESI: float = 3.0
    LOG_PATH: str = os.path.expanduser("~/Desktop/balloon_tracker")
    LOG_LEVEL: str = "DEBUG"
    ATIS_ESIK_PIKSEL: int = 10
    DYNAMIC_CONFIG: bool = False
    NAMLU_OFFSET_MM: float = 57.0
    PIXEL_PER_MM: float = 2.0
    VELOCITY_THRESHOLD: float = 5.0
    RASPI_IP: str = "192.168.1.100"
    RASPI_PORT: int = 8080
    TCP_TIMEOUT: float = 2.0
    TCP_SEND_INTERVAL: float = 1.0
    
    # MPU6050 ayarları
    MPU6050_ADDRESS: int = 0x68
    MPU6050_BUS: int = 1
    USE_MPU6050: bool = True
    ORIENTATION_FILTER_ALPHA: float = 0.96
    
    # Yön düzeltme katsayıları (derece/piksel)
    PITCH_TO_PIXEL_RATIO: float = 1.5  # Pitch değişimi başına piksel değişimi
    ROLL_TO_PIXEL_RATIO: float = 1.5   # Roll değişimi başına piksel değişimi


class EnhancedLogger:
    def __init__(self, config: Config):
        self.config = config
        self._setup_logging()

    def _setup_logging(self) -> None:
        os.makedirs(self.config.LOG_PATH, exist_ok=True)
        log_file = os.path.join(
            self.config.LOG_PATH,
            f"tracker_{datetime.now().strftime('%Y%m%d_%H%M%S')}.log"
        )

        logging.root.handlers = []
        formatter = logging.Formatter(
            fmt='%(asctime)s.%(msecs)03d - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

        handlers = []
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)

        console_handler = logging.StreamHandler()
        console_handler.setFormatter(formatter)
        handlers.append(console_handler)

        logging.basicConfig(
            level=getattr(logging, self.config.LOG_LEVEL),
            handlers=handlers
        )
        self.log("Logger initialized", "INFO", {"config": vars(self.config)})

    def log(self, message: str, level: str, extra_data: Optional[Dict[str, Any]] = None) -> None:
        log_entry = {
            "timestamp": datetime.now().isoformat(),
            "message": message,
            "level": level,
            **({"data": extra_data} if extra_data else {})
        }
        try:
            log_msg = json.dumps(log_entry, ensure_ascii=False, default=str)
            getattr(logging, level.lower())(log_msg)
        except Exception as e:
            logging.error(f"Logging error: {str(e)} - Original: {message}")


class MPUEnhancedKalmanFilter:
    """MPU6050 verilerini de içeren gelişmiş Kalman filtresi"""
    def __init__(self, config: Config):
        self.config = config
        # 8 durumlu Kalman filtresi: [x, y, vx, vy, ax, ay, pitch_offset, roll_offset]
        self.kalman = cv2.KalmanFilter(8, 4)  # 4 ölçüm: x, y, pitch, roll
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0, 0, 0, 0, 0],  # x
            [0, 1, 0, 0, 0, 0, 0, 0],  # y
            [0, 0, 0, 0, 0, 0, 1, 0],  # pitch offset
            [0, 0, 0, 0, 0, 0, 0, 1]   # roll offset
        ], np.float32)
        
        self.kalman.statePost = np.zeros((8, 1), dtype=np.float32)
        self.tahmin_gecmisi = deque(maxlen=20)
        self.measurement_history = deque(maxlen=10)
        self.guven_skoru = 0.0
        self.consecutive_good_measurements = 0
        self.measurement_variance = 0.0
        self.last_update_time = time.time()
        self.last_position = None
        self.velocity_magnitude = 0.0
        self.acceleration_magnitude = 0.0
        self.base_process_noise = config.KALMAN_PROCESS_NOISE
        self.base_measurement_noise = config.KALMAN_MEASUREMENT_NOISE
        self.is_initialized = False
        self.max_velocity = 200.0
        self.max_acceleration = 100.0
        self._initialize_matrices()

    def _initialize_matrices(self):
        dt = 1.0 / 30.0
        # Geçiş matrisi - yön bilgileri de dahil
        self.kalman.transitionMatrix = np.array([
            [1, 0, dt, 0, 0.5*dt*dt, 0, 0, 0],
            [0, 1, 0, dt, 0, 0.5*dt*dt, 0, 0],
            [0, 0, 1, 0, dt, 0, 0, 0],
            [0, 0, 0, 1, 0, dt, 0, 0],
            [0, 0, 0, 0, 1, 0, 0, 0],
            [0, 0, 0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 0, 0, 1, 0],  # pitch offset
            [0, 0, 0, 0, 0, 0, 0, 1]   # roll offset
        ], np.float32)
        
        # Süreç gürültüsü
        self.kalman.processNoiseCov = np.eye(8, dtype=np.float32)
        self.kalman.processNoiseCov[0:2, 0:2] *= self.base_process_noise
        self.kalman.processNoiseCov[2:4, 2:4] *= self.base_process_noise * 2
        self.kalman.processNoiseCov[4:6, 4:6] *= self.base_process_noise * 5
        self.kalman.processNoiseCov[6:8, 6:8] *= self.base_process_noise * 0.1  # Yön değişimi daha az gürültülü
        
        # Ölçüm gürültüsü
        self.kalman.measurementNoiseCov = np.eye(4, dtype=np.float32)
        self.kalman.measurementNoiseCov[0:2, 0:2] *= self.base_measurement_noise
        self.kalman.measurementNoiseCov[2:4, 2:4] *= self.base_measurement_noise * 0.5  # MPU verileri daha güvenilir
        
        self.kalman.errorCovPost = np.eye(8, dtype=np.float32) * 1000

    def guncelle_mpu_ile(self, pixel_olcum: Tuple[int, int], orientation_data: Dict[str, float]) -> Tuple[int, int]:
        """Hem piksel koordinatları hem de MPU yön verilerini kullanarak güncelle"""
        try:
            current_time = time.time()
            dt = current_time - self.last_update_time
            dt = max(0.01, min(dt, 0.1))
            
            # Pitch ve roll'u piksel offset'e çevir
            pitch_offset = orientation_data['pitch'] * self.config.PITCH_TO_PIXEL_RATIO
            roll_offset = orientation_data['roll'] * self.config.ROLL_TO_PIXEL_RATIO
            
            if not self.is_initialized:
                # İlk başlatma
                self.kalman.statePost[0, 0] = float(pixel_olcum[0])
                self.kalman.statePost[1, 0] = float(pixel_olcum[1])
                self.kalman.statePost[6, 0] = pitch_offset
                self.kalman.statePost[7, 0] = roll_offset
                self.is_initialized = True
                self.last_update_time = current_time
                self.last_position = pixel_olcum
                return pixel_olcum
            
            # Geçiş matrisini güncelle
            self._update_transition_matrix(dt)
            
            # Tahmin yap
            prediction = self.kalman.predict()
            
            # Ölçüm vektörü: [x, y, pitch_offset, roll_offset]
            measurement = np.array([
                [np.float32(pixel_olcum[0])],
                [np.float32(pixel_olcum[1])],
                [np.float32(pitch_offset)],
                [np.float32(roll_offset)]
            ])
            
            # Kalman güncelleme
            self.kalman.correct(measurement)
            
            # Durumu sınırla
            self._validate_and_constrain_state()
            
            # Güven skorunu güncelle
            measurement_quality = self._calculate_measurement_quality(pixel_olcum)
            self.guven_skoru = min(1.0, self.guven_skoru + (0.1 * measurement_quality))
            
            self.last_position = pixel_olcum
            self.last_update_time = current_time
            
            # MPU düzeltmeli pozisyon döndür
            return self._get_mpu_corrected_position()
            
        except Exception as e:
            logger.log("MPU-Kalman filtresi güncelleme hatası", "ERROR", {"error": str(e)})
            return pixel_olcum

    def _get_mpu_corrected_position(self) -> Tuple[int, int]:
        """MPU verilerini kullanarak düzeltilmiş pozisyon döndür"""
        try:
            state = self.kalman.statePost
            
            # Temel pozisyon
            x = state[0, 0]
            y = state[1, 0]
            
            # MPU düzeltmeleri
            pitch_correction = state[6, 0] 
            roll_correction = state[7, 0]
            
            # Düzeltmeleri uygula
            corrected_x = int(x + roll_correction)
            corrected_y = int(y + pitch_correction)
            
            # Sınırları kontrol et
            corrected_x = max(0, min(corrected_x, self.config.CAMERA_WIDTH))
            corrected_y = max(0, min(corrected_y, self.config.CAMERA_HEIGHT))
            
            corrected_position = (corrected_x, corrected_y)
            self.tahmin_gecmisi.append(corrected_position)
            
            return corrected_position
            
        except Exception as e:
            logger.log("MPU düzeltmeli pozisyon hesaplama hatası", "ERROR", {"error": str(e)})
            if self.tahmin_gecmisi:
                return self.tahmin_gecmisi[-1]
            return (self.config.CAMERA_WIDTH // 2, self.config.CAMERA_HEIGHT // 2)

    def _update_transition_matrix(self, dt):
        """Geçiş matrisini dt ile güncelle"""
        dt = max(0.001, min(dt, 0.1))
        self.kalman.transitionMatrix[0, 2] = dt
        self.kalman.transitionMatrix[0, 4] = 0.5 * dt * dt
        self.kalman.transitionMatrix[1, 3] = dt
        self.kalman.transitionMatrix[1, 5] = 0.5 * dt * dt
        self.kalman.transitionMatrix[2, 4] = dt
        self.kalman.transitionMatrix[3, 5] = dt

    def _calculate_measurement_quality(self, measurement: Tuple[int, int]) -> float:
        """Ölçüm kalitesini hesapla"""
        if len(self.measurement_history) < 2:
            return 0.5
        
        self.measurement_history.append(measurement)
        recent_positions = list(self.measurement_history)[-3:]
        
        if len(recent_positions) >= 2:
            positions = np.array(recent_positions)
            variance = np.var(positions, axis=0)
            self.measurement_variance = np.mean(variance)
            quality = 1.0 / (1.0 + self.measurement_variance / 100.0)
            return min(1.0, max(0.1, quality))
        return 0.5

    def _validate_and_constrain_state(self):
        """Durum vektörünü sınırla"""
        state = self.kalman.statePost
        state[0, 0] = max(-50, min(state[0, 0], self.config.CAMERA_WIDTH + 50))
        state[1, 0] = max(-50, min(state[1, 0], self.config.CAMERA_HEIGHT + 50))
        state[2, 0] = max(-self.max_velocity, min(state[2, 0], self.max_velocity))
        state[3, 0] = max(-self.max_velocity, min(state[3, 0], self.max_velocity))
        state[4, 0] = max(-self.max_acceleration, min(state[4, 0], self.max_acceleration))
        state[5, 0] = max(-self.max_acceleration, min(state[5, 0], self.max_acceleration))
        # Yön düzeltmelerini sınırla (-50 ile +50 piksel arası)
        state[6, 0] = max(-50, min(state[6, 0], 50))
        state[7, 0] = max(-50, min(state[7, 0], 50))
        self.kalman.statePost = state

    def tahmin_et(self) -> Tuple[int, int]:
        """Tahmin yap"""
        try:
            if not self.is_initialized:
                return (self.config.CAMERA_WIDTH // 2, self.config.CAMERA_HEIGHT // 2)
            
            current_time = time.time()
            dt = current_time - self.last_update_time
            dt = max(0.01, min(dt, 0.2))
            
            self._update_transition_matrix(dt)
            prediction = self.kalman.predict()
            self._validate_and_constrain_state()
            
            # Güven skorunu azalt
            confidence_decay = 0.05 * (dt / 0.033)
            self.guven_skoru = max(0.0, self.guven_skoru - confidence_decay)
            
            return self._get_mpu_corrected_position()
            
        except Exception as e:
            logger.log("MPU-Kalman filtresi tahmin hatası", "ERROR", {"error": str(e)})
            if self.tahmin_gecmisi:
                return self.tahmin_gecmisi[-1]
            return (self.config.CAMERA_WIDTH // 2, self.config.CAMERA_HEIGHT // 2)

    def get_tracking_stats(self) -> Dict[str, Any]:
        """Takip istatistiklerini döndür"""
        return {
            "confidence": self.guven_skoru,
            "velocity_magnitude": self.velocity_magnitude,
            "acceleration_magnitude": self.acceleration_magnitude,
            "measurement_variance": self.measurement_variance,
            "consecutive_good_measurements": self.consecutive_good_measurements,
            "prediction_history_length": len(self.tahmin_gecmisi),
            "is_initialized": self.is_initialized,
            "pitch_offset": float(self.kalman.statePost[6, 0]) if self.is_initialized else 0.0,
            "roll_offset": float(self.kalman.statePost[7, 0]) if self.is_initialized else 0.0
        }

    def reset(self):
        """Filtresi sıfırla"""
        self.kalman.statePost = np.zeros((8, 1), dtype=np.float32)
        self.kalman.errorCovPost = np.eye(8, dtype=np.float32) * 1000
        self.tahmin_gecmisi.clear()
        self.measurement_history.clear()
        self.guven_skoru = 0.0
        self.consecutive_good_measurements = 0
        self.is_initialized = False
        self.last_position = None
        self._initialize_matrices()


class TCPCommunicator:
    def __init__(self, config: Config):
        self.config = config
        self.socket = None
        self.lock = threading.Lock()
        self.connected = False
        self.connection_attempts = 0
        self.max_connection_attempts = 5
        self.last_connection_attempt = 0
        self.connection_retry_interval = 5.0

    def connect(self) -> bool:
        current_time = time.time()
        if (current_time - self.last_connection_