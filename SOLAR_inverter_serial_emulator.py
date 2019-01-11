
import serial
import time
import random 

# ОПРЕДЕЛЕНИЕ СОМ-ПОРТА ДЛЯ ОБМЕНА С ИНВЕРТОРОМ

ser = serial.Serial(              
    port='COM7',
    baudrate = 2400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# КОМАНДЫ ДЛЯ ИНВЕРТОРА + СRC16

QPGS0 = '\x51\x50\x47\x53\x30\x3F\xDa\x0D'                      # Parallel Information inquiry（For 4K/5K）
QPIGS = '\x51\x50\x49\x47\x53\xB7\xA9\x0D'                      # Device general status parameters inquiry
QMCHGCR = '\x51\x4D\x43\x48\x47\x43\x52\xD8\x55\x0D'            # Enquiry selectable value about max charging current
QMUCHGCR = '\x51\x4D\x55\x43\x48\x47\x43\x52\x26\x34\0D'        # Enquiry selectable value about max utility charging current
QPIWS = '\x51\x50\x49\x57\x53\xB4\xDA\x0D'                      # Device Warning Status inquiry
QMOD = '\x51\x4D\x4F\x44\x49\xC1\x0D'                           # Device Mode inquiry
QDI = '\x51\x44\x49\x71\x1B\x0D'                                # The default setting value information
QVFW = '\x51\x56\x46\x57\x62\x99\x0D'                           # Main CPU Firmware version inquiry
QVFW2 = '\x51\x56\x46\x57\x32\xC3\xF5\x0D'                      # Another CPU Firmware version inquiry
QPIRI = '\x51\x50\x49\x52\x49\xF8\x54\x0D'                      # Device Rating Information inquiry
QFLAG = '\x51\x46\x4C\x41\x47\x98\x74\x0D'                      # Device flag status inquiry 
# ACK = '\x41\x43\x4B\x39\x20\x0D'                                # Device response
POP = {'UTI':'\x50\x4F\x50\x30\x30\xC2\x48\x0D',                # Setting device output source priority to UTI
       'SOL':'\x50\x4F\x50\x30\x31\xD2\x69\x0D',                # Setting device output source priority to SOL
       'SBU':'\x50\x4F\x50\x30\x32\xE2\x0B\x0D'                 # Setting device output source priority to SBU
       }
PCP = {'UTI':'\x50\x43\x50\x30\x30\x8D\x7A\x0D',                # Setting device charger priority to UTI first
       'SOL':'\x50\x43\x50\x30\x31\x9D\x5B\x0D',                # Setting device charger priority to SOL first
       'SOL+UTI':'\x50\x43\x50\x30\x32\xAD\x38\x0D',            # Setting device charger priority to SOL+UTI
       'OnlySOL':'\x50\x43\x50\x30\x33\xBD\x19\x0D'             # Setting device charger priority to OnlySOL
       }
PGR = {'APP':'\x50\x47\x52\x30\x30\x29\xEB\x0D',                # Setting device grid working range to APP
       'UPS':'\x50\x47\x52\x30\x31\x39\xCA\x0D'                 # Setting device grid working range to UPS
       }                      
PBCV = {'44.0':'\x50\x42\x43\x56\x34\x34\x2E\x30\xE6\xEB\x0D',  # Set battery re-charge voltage to 44.0 V
        '45.0':'\x50\x42\x43\x56\x34\x35\x2E\x30\xD1\xDB\x0D',  # Set battery re-charge voltage to 45.0 V
        '46.0':'\x50\x42\x43\x56\x34\x36\x2E\x30\x88\x8B\x0D',  # Set battery re-charge voltage to 46.0 V
        '47.0':'\x50\x42\x43\x56\x34\x37\x2E\x30\xBF\xBB\x0D',  # Set battery re-charge voltage to 47.0 V
        '48.0':'\x50\x42\x43\x56\x34\x38\x2E\x30\x93\x8A\x0D',  # Set battery re-charge voltage to 48.0 V
        '49.0':'\x50\x42\x43\x56\x34\x39\x2E\x30\xA4\xBA\x0D',  # Set battery re-charge voltage to 49.0 V
        '50.0':'\x50\x42\x43\x56\x35\x30\x2E\x30\x4C\x9F\x0D',  # Set battery re-charge voltage to 50.0 V
        '51.0':'\x50\x42\x43\x56\x35\x31\x2E\x30\x7B\xAF\x0D'   # Set battery re-charge voltage to 51.0 V
        }
PSDV = {'40.0':'\x50\x53\x44\x56\x34\x30\x2E\x30  <crc> \x0D',  # Set battery under voltage to 40.0 V
        '41.0':'\x50\x53\x44\x56\x34\x31\x2E\x30  <crc> \x0D',  # Set battery under voltage to 41.0 V
        '42.0':'\x50\x53\x44\x56\x34\x32\x2E\x30  <crc> \x0D',  # Set battery under voltage to 42.0 V
        '43.0':'\x50\x53\x44\x56\x34\x33\x2E\x30  <crc> \x0D',  # Set battery under voltage to 43.0 V
        '44.0':'\x50\x53\x44\x56\x34\x34\x2E\x30  <crc> \x0D',  # Set battery under voltage to 44.0 V
        '45.0':'\x50\x53\x44\x56\x34\x35\x2E\x30  <crc> \x0D',  # Set battery under voltage to 45.0 V
        '46.0':'\x50\x53\x44\x56\x34\x36\x2E\x30  <crc> \x0D',  # Set battery under voltage to 46.0 V
        '47.0':'\x50\x53\x44\x56\x34\x37\x2E\x30  <crc> \x0D',  # Set battery under voltage to 47.0 V
        '48.0':'\x50\x53\x44\x56\x34\x38\x2E\x30  <crc> \x0D'   # Set battery under voltage to 48.0 V
        }
PBDV = {'48.0':'\x50\x42\x44\x56\x34\x38\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 48.0 V
        '49.0':'\x50\x53\x44\x56\x34\x39\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 49.0 V
        '50.0':'\x50\x53\x44\x56\x35\x30\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 50.0 V
        '51.0':'\x50\x53\x44\x56\x35\x31\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 51.0 V
        '52.0':'\x50\x53\x44\x56\x35\x32\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 52.0 V
        '53.0':'\x50\x53\x44\x56\x35\x33\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 53.0 V
        '54.0':'\x50\x53\x44\x56\x35\x34\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 54.0 V
        '55.0':'\x50\x53\x44\x56\x35\x35\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 55.0 V
        '56.0':'\x50\x53\x44\x56\x35\x36\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 56.0 V
        '57.0':'\x50\x53\x44\x56\x35\x37\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 57.0 V
        '58.0':'\x50\x53\x44\x56\x35\x38\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 58.0 V        
        }
        

def crc16(message):
    
# РАСЧЕТ СRC16 по алгоритму CRC-CCITT (XModem)
# https://bytes.com/topic/python/insights/887357-python-check-crc-frame-crc-16-ccitt      
# CRC-16-CITT poly, the CRC sheme used by ymodem protocol
# 16bit operation register, initialized to zeros
# message - строка данных, для которой расчитывается CRC
# функция возвращает строку вида '\x00\x00', соответствующую 2-м символам СRC16

    poly = 0x1021
    reg = 0
    #pad the end of the message with the size of the poly
    message += '\x00\x00' 
    #for each bit in the message
    for byte in message:
        mask = 0x80
        while(mask > 0):
            #left shift by one
            reg<<=1
            #input the next bit from the message into the right hand side of the op reg
            if ord(byte) & mask:   
                reg += 1
            mask>>=1
            #if a one popped out the left of the reg, xor reg w/poly
            if reg > 0xffff:            
                #eliminate any one that popped out the left
                reg &= 0xffff           
                #xor with the poly, this is the remainder
                reg ^= poly
            crc_l = reg & 0xFF
            crc_h = reg >> 8            
    return chr(crc_h) + chr(crc_l)

qmod = '(P'
i = 0
ct = 0

# ПРОСЛУШИВАНИЕ СОМ-ПОРТА И ЭМУЛЯЦИЯ ОТВЕТА ИНВЕРТОРА

while True :
    
    rd_serial = ser.readline().decode('utf-8')
    data = rd_serial[:-3]
    crc = rd_serial[-3:-1]

    if crc == crc16(data) :
        print('read serial :', rd_serial)
        print('       data =', data, '  crc =', crc)
        
         if data == 'QMOD':
            if (qmod == '(P') & (ct >= 4): 
                qmod = '(S'
                ct = 0
            elif qmod == '(S') & (ct >= 4): 
                qmod = '(L'
                ct = 0
            elif qmod == '(L') & (ct >= 4): 
                qmod = '(B'
                ct = 0
            elif qmod == '(B') & (ct >= 4): 
                qmod = '(F'
                ct = 0
            elif qmod == '(F') & (ct >= 4): 
                qmod = '(H'
                ct = 0
            elif qmod == '(H') & (ct >= 4):
                qmod = '(P'
                ct = 0
            else : 
                qmod = '(B'
                ct = 0
            answer = qmod 
            ct =+ 1
 
        if data == 'QPIGS':
            grid_voltage = 220.0 + random.uniform(-20, 20)
            grid_frequence = 50.0 + random.uniform(-2, 2)
            
            if qmod == '(B' : ac_output_voltage = 230.0 + random.uniform(-2, 2)
            if qmod == '(L' : ac_output_voltage = grid_voltage
            else : ac_voltage = 0.0
        
            if qmod == '(B' : ac_output_frequnce = 50.0 + random.uniform(-1, 1)
            if qmod == '(L' : ac_output_frequnce = grid_frequency
            else : ac_frequency = 0.0
           
            output_load_percent = 75 + random.randint(-25, 25)
            ac_output_apparent_power = output_load_percent * 40
            ac_output_active_power = int(ac_output_apparent_power * 0.85)
            bus_voltage = 420 + random.randint(-5, 5)
            battery_voltage = 48.00 + random.uniform(-6, 6)
            battery_charging_current = 100 + random.randint(-50, 50)
            battery_capacity = int(100 * battery_voltage / 56.0)
            inverter_temperature = 20 + random.randint(-5, 15)
            pv_input_current = 15 + random.randint(-10, 5)
            pv_input_voltage = 72.0 + random.uniform(-20, 10)
            scc_voltage = battery_voltage + 2
            discharge_current = 100 + random.randint(-20, 20)
            device_status = ' 00010111'
            
            answer = '('+'{:0>4.1f}'.format(grid_voltage)\
                     +' {:0>3.1f}'.format(grid_frequence)\
                     +' {:0>4.1f}'.format(ac_output_voltage)\
                     +' {:0>3.1f}'.format(ac_output_frequnce)\
                     +' {:0>4d}'.format(ac_output_apparent_power)\
                     +' {:0>4d}'.format(ac_output_active_power)\
                     +' {:0>3d}'.format(output_load_percent)\
                     +' {:0>3d}'.format(bus_voltage)\
                     +' {:0>5.2f}'.format(battery_voltage)\
                     +' {:0>3d}'.format(battery_charging_current)\
                     +' {:0>3d}'.format(battery_capacity)\
                     +' {:0>4d}'.format(inverter_temperature)\
                     +' {:0>4d}'.format(pv_input_current)\
                     +' {:0>5.1f}'.format(pv_input_voltage)\
                     +' {:0>5.2f}'.format(scc_voltage)\
                     +' {:0>5d}'.format(discharge_current)\
                     + device_status\
                     +' 00 00 00000 100'
            
        if data == 'QPIRI':            
            answer = '(230.0 21.7 230.0 50.0 21.7 5000 4000 48.0 46.0 42.0 56.4 54.0 0 10 010 1 0 0 6 01 0 0 54.0 0 1'
            
            
        if data == 'QPIWS':

            if i == 0b00000000000000000000000000000000 : i =+ 1
            else : i = (i << 1) + 1
            if i > 0b00111111111111111111111111111111 : i = 0
            
            answer ='({:0>32b}'.format(i)
            
        msg_wr = answer + crc16(answer)+'\x0D'
        print('write serial :', msg_wr)
        print('        data =', answer, '  crc =', msg_wr[-3:-1])
        ser.write(bytes(msg_wr, 'utf-8'))

    else : print('read serial...')
