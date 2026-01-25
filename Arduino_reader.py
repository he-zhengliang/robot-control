import serial
import serial.tools.list_ports
import time
import threading
import sys

# ---------------- CONFIG ----------------
ARDUINO_PORT = '/dev/ttyACM1'  # 建议检查: ls /dev/ttyACM*
ARDUINO_BAUD = 115200         # 建议调至 115200 观察稳定性

GRIPPER_PORT = '/dev/ttyACM0'
GRIPPER_BAUD = 9600
# ----------------------------------------

class ExperimentController:
    def __init__(self):
        self.arduino = None
        self.gripper = None
        self.running = True
        self.last_data = ""

    def connect_all(self):
        """初始化所有连接，带重置逻辑"""
        try:
            # 连接 Arduino
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=0.1)
            # 触发 DTR 复位 (模拟 Arduino IDE)
            self.arduino.setDTR(False)
            time.sleep(0.1)
            self.arduino.reset_input_buffer()
            self.arduino.setDTR(True)
            print(f"[SUCCESS] Arduino connected on {ARDUINO_PORT}")

            # 连接 Gripper
            self.gripper = serial.Serial(GRIPPER_PORT, GRIPPER_BAUD, timeout=0.1)
            self.gripper.reset_input_buffer()
            print(f"[SUCCESS] Gripper connected on {GRIPPER_PORT}")
            
            print("[INFO] Waiting 2s for hardware self-test...")
            time.sleep(2)
            return True
        except Exception as e:
            print(f"[ERROR] Connection failed: {e}")
            return False

    def send_gripper_hex(self, hex_str):
        """带检查的十六进制发送"""
        if self.gripper and self.gripper.is_open:
            try:
                cmd = bytes.fromhex(hex_str.replace(" ", ""))
                self.gripper.write(cmd)
                self.gripper.flush()
                time.sleep(0.1)
            except Exception as e:
                print(f"[ERROR] Gripper Send Error: {e}")

    def gripper_sequence(self):
        """执行实验动作序列"""
        print("[ACTION] Starting sequence...")
        # 1. 开手
        self.send_gripper_hex('AA 55 01 00 00 00 00 05 FF 03 FF 00 9F 02 9A')
        time.sleep(2)
        # 2. 闭合拇指
        self.send_gripper_hex('AA 55 01 00 00 00 00 08 FF 05 96 00 90 01 F4')
        time.sleep(1)
        # 3. 闭合手指
        self.send_gripper_hex('AA 55 00 01 01 01 01 08 FF 02 58 01 20 01 8D')
        time.sleep(3)
        # 4. 释放
        self.send_gripper_hex('AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')
        print("[ACTION] Sequence finished.")

    def read_loop(self):
        """持续读取线程"""
        print("[THREAD] Arduino reader started.")
        while self.running:
            try:
                if self.arduino.in_waiting > 0:
                    line = self.arduino.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 此处可以扩展为解析 COP 坐标
                        # x, y = self.parse_cop(line)
                        print(f"RAW: {line}")
                else:
                    time.sleep(0.01) # 防止 CPU 占用过高
            except Exception as e:
                print(f"[DEBUG] Read error: {e}")
                time.sleep(0.1)

    def cleanup(self):
        self.running = False
        time.sleep(0.5)
        if self.arduino: self.arduino.close()
        if self.gripper: self.gripper.close()
        print("[INFO] Resources cleaned up.")

def main():
    exp = ExperimentController()
    
    if not exp.connect_all():
        sys.exit(1)

    # 启动后台读取
    read_thread = threading.Thread(target=exp.read_loop, daemon=True)
    read_thread.start()

    try:
        # 执行动作
        exp.gripper_sequence()
        
        # 保持主程序运行，直到用户按下 Ctrl+C
        print("\n[READY] Capturing data. Press Ctrl+C to stop.")
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[INFO] User interrupted.")
    finally:
        exp.cleanup()

if __name__ == "__main__":
    main()
