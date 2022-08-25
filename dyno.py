import sys
import time
import csv
import datetime
import serial
import math
from daqhats import hat_list, HatIDs, mcc118
import RPi.GPIO as GPIO



def init_daqhat():
    # Initializes daqhat (MCC 118), returns board address
    board_list = hat_list(filter_by_id = HatIDs.ANY)
    
    if not board_list:
        print("No boards found")
        sys.exit()
    
    for entry in board_list:
        if entry.id == HatIDs.MCC_118:
            print("Board {}: MCC 118".format(entry.address))
            board = mcc118(entry.address)
            
    return board



def init_GPIO():
    # Initializes RPi GPIO channels
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(4, GPIO.IN)
        


def  init_csv(rpm, direction):
    # Writes CSV header and returns directory
    header = ['Brake Voltage (V)', 'Motor Voltage (V)', 'Current (A)', 'Torque (N.m)', 'Rot Speed (RPM)', 'Electrical Power (W)', 'Mecanical Power (W)', 'Efficiency (%)']
    
    time = datetime.datetime.now()
    directory = '/home/hefring/Desktop/test_files/' + time.strftime('%Y-%m-%d__%H-%M-%S__' + str(rpm) + 'RPM_' + direction + '.csv')
    
    with open(directory, 'w', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
    
    return directory



def init_brake(com):
    # Initializes power supply for the brake
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/tty' + com
    
    ser.open()
    ser.write(bytes('OUT1', 'UTF-8')) # Turn on supply output
    time.sleep(0.1)
    ser.write(bytes('VSET1:0', 'UTF-8')) # Sets voltage to 0V
    ser.close()
    

        
def write_to_csv(directory, data):
    # Writes data to CSV file
    # data = [brake, volt, curr, torque, rpm, Epower, Mpower, eff]
    
    with open(directory, 'a', encoding='UTF8', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(data)
        
        
        
def get_voltage(board, channel):
    # Reads and return voltage from daqhat's (MCC 118) selected channel
    volt_div = 247/1369 # Voltage divider value
    voltage = 0
    
    for i in range(500):
        voltage += board.a_in_read(channel)
        
    voltage /= i+1
    voltage /= volt_div
    return voltage



def get_current(board, channel):
    # Reads and return current from daqhat's (MCC 118) selected channel
    current = 0
    
    for i in range(500):
        current += board.a_in_read(channel)
        time.sleep(0.0000015)
        
    current /= i+1
    current -= 0.334 # measurement offset @rest
    current /= 0.264 # current sensor: 264mV/A
    return current


def get_torque(board, channel):
    # Reads and return torque from daqhat's (MCC 118) selected channel
    torque = 0
    
    for i in range(500):
        torque += board.a_in_read(channel)
    
    torque /= i+1
    torque *= 2 # torque transducer outputs [-10V;10V] for [-20N.m;20N.m]
    return torque



def get_speed(channel):
    # Calculates and return speed from hall sensor and RPi GPIO input
    revs = 10 # Number of rotation to perform
    counter = 0
    previous = 1
    state = 1
    flag = True

    while(counter <= revs):
        time.sleep(0.05)
        previous = state
        state = GPIO.input(channel)
        
        if (state == 0 and previous == 1):
            
            if flag:
                start = time.perf_counter_ns()
                flag = False
                
            counter += 1
            
    stop = time.perf_counter_ns()
    elapsed_time = (stop - start)/1000000000
    rpm = 60 * revs / elapsed_time
    
    return rpm



def start_motor(com):
    # Gradually starts motor at desired speed and direction
    ratio = 9 # Motor reducer ratio
    
    ser = serial.Serial()
    ser.baudrate = 115200
    
    ser.port = '/dev/tty' + com
    
    motor_dir = ""
    while not(motor_dir == "up" or motor_dir == "u" or motor_dir == "down" or motor_dir == "d"):
        motor_dir = input("Enter motor direction (up (U) / down (D)): ").lower()
    
    cmd = 0  # Speed command
    checksum = 0  # Last byte in packet (verifier)
    
    rpm = int(input("Enter speed (RPM): "))
    value = 10
    
    if rpm != 0:
        
        while (value < rpm):
            
            packet = bytearray()
            packet.append(0x3E)
            packet.append(0xA2)  # 0xA2: speed control
            packet.append(0x01)
            packet.append(0x04)
            packet.append(0xE5)
            
            cmd = 100 * 6 * ratio * value
            if (motor_dir == "up" or motor_dir == "u"):
                cmd = cmd ^ 0xFFFFFFFF
                
            cmd_byte = cmd.to_bytes(4, byteorder='little')
            packet.extend(cmd_byte)
        
            checksum = sum(cmd_byte)
            checksum_byte = checksum.to_bytes(2, byteorder='big')
            packet.append(checksum_byte[-1])
            
            ser.open()
            ser.write(packet)
            print(packet)
            ser.close()
            
            value += 15
            time.sleep(2)
            
        packet = bytearray()
        packet.append(0x3E)
        packet.append(0xA2)  # 0xA2: speed control
        packet.append(0x01)
        packet.append(0x04)
        packet.append(0xE5)
        
        cmd = 100 * 6 * ratio * rpm
        if (motor_dir == "up" or motor_dir == "u"):
            cmd = cmd ^ 0xFFFFFFFF
            motor_dir = 'UP'
            
        else:
            motor_dir = 'DOWN'
            
        cmd_byte = cmd.to_bytes(4, byteorder='little')
        packet.extend(cmd_byte)
    
        checksum = sum(cmd_byte)
        checksum_byte = checksum.to_bytes(2, byteorder='big')
        packet.append(checksum_byte[-1])
        
        ser.open()
        ser.write(packet)
        print(packet)
        ser.close()
        
        return (rpm, motor_dir)
        
        
        
def stop_motor(com):
    # Stops motor
    ser = serial.Serial()
    ser.baudrate = 115200
    
    ser.port = '/dev/tty' + com

    packet_stop = bytearray()
    packet_stop.append(0x3E)
    packet_stop.append(0xA1)  # 0xA1: torque control
    packet_stop.append(0x01)
    packet_stop.append(0x02)
    packet_stop.append(0xE2)
    packet_stop.append(0x00)
    packet_stop.append(0x00) 
    packet_stop.append(0x00)

    ser.open()
    ser.write(packet_stop)
    print(packet_stop)
    ser.close()



def apply_brake_voltage(com, volt):
    # Apply brake voltage
    ser = serial.Serial()
    ser.baudrate = 9600
    ser.port = '/dev/tty' + com
    
    ser.open()
    ser.write(bytes('VSET1:' + str(volt), 'UTF-8'))
    ser.close()
    
    

def main():
    # Set channels
    # Set brake voltage increase step
    
    motor_com = input("Enter motor COM Port #: ")
    brake_com = input("Enter brake COM Port #: ")
    
    voltage_channel = 5
    current_channel = 6
    torque_channel = 4
    speed_channel = 4
    
    brake_voltage = 0
    brake_increase = 0.5 # Step size to increase brake voltage
    Mpower = 0
    Epower = 0
    eff = 0
    
    board = init_daqhat()
    init_GPIO()
    init_brake(brake_com)
    
    (rpm, direction) = start_motor(motor_com)
    print('Motor started')
    directory = init_csv(rpm, direction)
    time.sleep(2)
    
    try:
        while(brake_voltage <= 24): # Needs another condition to check if the motor is still running
            apply_brake_voltage(brake_com, brake_voltage)
            time.sleep(2)
            
            print('Measuring...\n')
            rpm = get_speed(speed_channel)
            print('Speed updated')
            torque = get_torque(board, torque_channel)
            print('Torque updated: ', torque)
            voltage = get_voltage(board, voltage_channel)
            print('Voltage updated')
            current = get_current(board, current_channel)
            print('Current updated\n')
            
            Mpower = math.pi / 30 * rpm * abs(torque) # Mechanical Power
            Epower = voltage * current # Electrical Power
            eff = Mpower / Epower * 100
            
            data = [brake_voltage, voltage, current, torque, rpm, Epower, Mpower, eff]
            write_to_csv(directory, data)
            print('CSV updated\n ')
            
            brake_voltage += brake_increase
            time.sleep(2) 
            
        apply_brake_voltage(brake_com, 0)
        time.sleep(5)
        stop_motor(motor_com)
            
    except KeyboardInterrupt:
        stop_motor(motor_com)
        apply_brake_voltage(brake_com, 0)
        write_to_csv(directory, ['ABORTED'])
        print('ABORTED')



if __name__ == "__main__":
    main()