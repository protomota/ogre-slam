"""
Encoder Reader for Mecanum Drive Robot
Reads quadrature encoders via Jetson GPIO using interrupt-based counting
"""
import Jetson.GPIO as GPIO
import threading
from typing import List, Tuple

class EncoderReader:
    """
    Reads 4 quadrature encoders (2 channels each) from Jetson GPIO

    Hardware Configuration:
    - Motors: M1 (FL), M2 (FR), M3 (RL), M4 (RR)
    - Encoders: 2 PPR (pulses per revolution) Hall sensors
    - GPIO Mode: BOARD (physical pin numbering)
    """

    # Encoder pin mappings (BOARD mode - physical pin numbers)
    ENCODER_PINS = {
        1: (7, 11),    # M1 (Front-Left): ENC1A=Pin7, ENC1B=Pin11
        2: (13, 15),   # M2 (Front-Right): ENC2A=Pin13, ENC2B=Pin15
        3: (29, 31),   # M3 (Rear-Left): ENC3A=Pin29, ENC3B=Pin31
        4: (32, 33),   # M4 (Rear-Right): ENC4A=Pin32, ENC4B=Pin33
    }

    # Motor layout for mecanum drive
    # M4 (FL)  M1 (FR)
    # M3 (RL)  M2 (RR)

    def __init__(self):
        """Initialize encoder reader with GPIO setup"""
        self.lock = threading.Lock()

        # Encoder tick counts [M1, M2, M3, M4]
        self.ticks = [0, 0, 0, 0]

        # Last direction for each motor (1=forward, -1=backward)
        self.directions = [1, 1, 1, 1]

        # Setup GPIO
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)

        # Setup encoder pins
        self._setup_encoders()

    def _setup_encoders(self):
        """Setup GPIO pins and interrupts for all 4 encoders"""
        for motor_num in range(1, 5):
            pin_a, pin_b = self.ENCODER_PINS[motor_num]

            # Setup both pins as inputs
            GPIO.setup(pin_a, GPIO.IN)
            GPIO.setup(pin_b, GPIO.IN)

            # Add interrupts on rising edges
            # Using lambda with default argument to capture motor_num correctly
            GPIO.add_event_detect(
                pin_a,
                GPIO.RISING,
                callback=lambda channel, m=motor_num: self._encoder_callback(m, 'A'),
                bouncetime=1
            )

            GPIO.add_event_detect(
                pin_b,
                GPIO.RISING,
                callback=lambda channel, m=motor_num: self._encoder_callback(m, 'B'),
                bouncetime=1
            )

    def _encoder_callback(self, motor_num: int, channel: str):
        """
        Encoder interrupt callback

        Args:
            motor_num: Motor number (1-4)
            channel: 'A' or 'B'
        """
        with self.lock:
            idx = motor_num - 1  # Convert to 0-indexed
            # Increment tick count (direction will be applied during velocity calculation)
            self.ticks[idx] += 1

    def get_ticks(self) -> List[int]:
        """
        Get current encoder tick counts

        Returns:
            List of tick counts [M1, M2, M3, M4]
        """
        with self.lock:
            return self.ticks.copy()

    def reset_ticks(self):
        """Reset all encoder tick counts to zero"""
        with self.lock:
            self.ticks = [0, 0, 0, 0]

    def get_ticks_and_reset(self) -> List[int]:
        """
        Get current ticks and reset counters atomically

        Returns:
            List of tick counts [M1, M2, M3, M4] before reset
        """
        with self.lock:
            ticks_copy = self.ticks.copy()
            self.ticks = [0, 0, 0, 0]
            return ticks_copy

    def set_directions(self, directions: List[int]):
        """
        Set motor directions for signed tick counting

        Args:
            directions: List of directions [M1, M2, M3, M4] where 1=forward, -1=backward
        """
        with self.lock:
            self.directions = directions.copy()

    def get_signed_ticks_and_reset(self) -> List[int]:
        """
        Get signed tick counts (accounting for direction) and reset

        Returns:
            List of signed ticks [M1, M2, M3, M4]
        """
        with self.lock:
            signed_ticks = [t * d for t, d in zip(self.ticks, self.directions)]
            self.ticks = [0, 0, 0, 0]
            return signed_ticks

    def cleanup(self):
        """Cleanup GPIO resources"""
        try:
            GPIO.cleanup()
        except:
            pass

    def __del__(self):
        """Destructor - ensure GPIO cleanup"""
        self.cleanup()


if __name__ == "__main__":
    # Simple test
    import time

    print("Testing encoder reader...")
    print("Rotate motor wheels to see encoder counts")
    print("Press Ctrl+C to exit")

    reader = EncoderReader()

    try:
        while True:
            ticks = reader.get_ticks()
            print(f"Ticks: M1={ticks[0]:4d} M2={ticks[1]:4d} M3={ticks[2]:4d} M4={ticks[3]:4d}", end='\r')
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        reader.cleanup()
