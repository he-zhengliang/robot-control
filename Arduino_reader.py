import serial
import time
import threading
import sys

# ---------------- CONFIG ----------------
ARDUINO_PORT = '/dev/ttyACM1'  
ARDUINO_BAUD = 250000  # 建议降至 115200 提高抗干扰能力，若必须 250000 请修改此处

GRIPPER_PORT = '/dev/ttyACM0'  
GRIPPER_BAUD = 9600
# ----------------------------------------

def connect_device(port, baud, name):
    """安全连接设备并初始化"""
    try:
        print(f"[INFO] Connecting to {name} on {port}...")
        # rtscts=True 增加硬件流控稳定性
        ser = serial.Serial(port, baud, timeout=1, rtscts=True)
        
        # 核心：触发 Arduino 复位并清理缓存
        ser.setDTR(False)
        time.sleep(0.1)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        ser.setDTR(True)
        
        print(f"[SUCCESS] {name} connected. Waiting for reset...")
        time.sleep(2)  # 给硬件 2 秒启动时间
        return ser
    except Exception as e:
        print(f"[ERROR] Cannot connect to {name}: {e}")
        return None

def send_hex_command(ser, hex_command):
    """发送十六进制命令"""
    if ser and ser.is_open:
        try:
            hex_command = hex_command.replace(" ", "")
            hex_bytes = bytes.fromhex(hex_command)
            ser.write(hex_bytes)
            ser.flush() # 确保数据完全发出
            time.sleep(0.1)
        except Exception as e:
            print(f"[ERROR] Failed to send command: {e}")

def gripper_sequence(gripper_ser):
    """机械手动作序列"""
    if not gripper_ser: return
    try:
        print("[INFO] Starting gripper sequence...")
        # Step 1: Open hand
        send_hex_command(gripper_ser, 'AA 55 01 00 00 00 00 05 FF 03 FF 00 9F 02 9A')
        time.sleep(2)

        # Step 2: Close thumb
        send_hex_command(gripper_ser, 'AA 55 01 00 00 00 00 08 FF 05 96 00 90 01 F4')
        time.sleep(1)
        
        # Step 3: Close fingers
        send_hex_command(gripper_ser, 'AA 55 00 01 01 01 01 08 FF 02 58 01 20 01 8D')
        time.sleep(3)

        # Step 4: Open hand to release
        send_hex_command(gripper_ser, 'AA 55 00 01 01 01 01 05 FF 03 FF 01 2F 02 2A')
        print("[INFO] Gripper sequence completed.")
    except Exception as e:
        print("[ERROR] Gripper sequence interrupted:", e)

def read_arduino(arduino_ser):
    """连续读取 Arduino 数据，带容错处理"""
    if not arduino_ser: return
    print("[INFO] Reading Arduino data...")
    while True:
        try:
            if arduino_ser.in_waiting > 0:
                line = arduino_ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    # 在这里可以增加你的数据处理逻辑
                    print(f"Data: {line}")
        except Exception as e:
            print(f"[WARNING] Read error: {e}")
            time.sleep(0.1)

def main():
    # 1. 建立连接
    arduino_ser = connect_device(ARDUINO_PORT, ARDUINO_BAUD, "Arduino")
    gripper_ser = connect_device(GRIPPER_PORT, GRIPPER_BAUD, "Gripper")

    if not arduino_ser or not gripper_ser:
        print("[FATAL] Required devices not found. Exiting.")
        sys.exit(1)

    # 2. 开启读取线程
    arduino_thread = threading.Thread(target=read_arduino, args=(arduino_ser,), daemon=True)
    arduino_thread.start()

    # 3. 执行机械手动作
    try:
        gripper_sequence(gripper_ser)
        
        # 动作完成后保持读取状态
        print("[INFO] Sequence finished. Keeping reader alive (Ctrl+C to stop)...")
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[INFO] Stopped by user.")
    finally:
        # 4. 安全关闭
        if gripper_ser: gripper_ser.close()
        if arduino_ser: arduino_ser.close()
        print("[INFO] Serial connections closed.")

if __name__ == "__main__":
    main()
