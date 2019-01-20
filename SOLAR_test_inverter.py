#import 
import serial
import time
 


# ОПРЕДЕЛЕНИЕ СОМ-ПОРТА ДЛЯ ОБМЕНА С ИНВЕРТОРОМ

ser = serial.Serial(              
    port='COM3',
    baudrate = 2400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)

# КОМАНДЫ + СRC16 ДЛЯ ПЕРЕДАЧИ В ИНВЕРТОР 

QPGS0 = '\x51\x50\x47\x53\x30\x3F\xDa\x0D'                      # Parallel Information inquiry（For 4K/5K）
QPIGS = '\x51\x50\x49\x47\x53\xB7\xA9\x0D'                      # Device general status parameters inquiry
QMCHGCR = '\x51\x4D\x43\x48\x47\x43\x52\xD8\x55\x0D'            # Enquiry selectable value about max charging current
QMUCHGCR = '\x51\x4D\x55\x43\x48\x47\x43\x52\x26\x34\x0D'       # Enquiry selectable value about max utility charging current
QPIWS = '\x51\x50\x49\x57\x53\xB4\xDA\x0D'                      # Device Warning Status inquiry
QMOD = '\x51\x4D\x4F\x44\x49\xC1\x0D'                           # Device Mode inquiry
QID = '\x51\x49\x44\xD6\xEA\x0D'                                # The device serial number inquiry
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
        

set_time = 30                                                   # периодичность опроса инвертора xx, s
cmd = ''
info_rd = False


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
            crc_h = reg >> 8
            crc_l = reg & 0xFF    
    return chr(crc_h) + chr(crc_l)        


def comm_inverter(msg_wr):

# ОБМЕН С ИНВЕРТОРОМ ЧЕРЕЗ COM-ПОРТ
# msg_wr - строка данных для передачи инвертору через СОМ-порт
# data - строка данных, полученных от инвертора через СОМ-порт (без CRC и \r)
# crc - 2 символа, соответствующие CRC16 для данных, полученных от инвертора через СОМ-порт
# функция возвращает кортеж вида : <data> , <length>, <сrc> 

#    ser.write(bytes(msg_wr, 'utf-8'))
    print(msg_wr)

    time.sleep(0.5)
#    msg_rd = ser.readline().decode()
    msg_rd = input()
    
    data = msg_rd[:-3]
    length = len(data)
    crc = msg_rd[-3:-1]

#    if crc16(data) != crc : length = 0
#    return data, length, crc
    return msg_rd, length 

# ОТКРЫТИЕ ФАЙЛА

#file = open(r"C:\Users\Сергей\Documents\RaspberryPi\test.txt", "r")
#file = open(r"C:\Users\Сергей\Documents\RaspberryPi\test.txt", "r", closefd=True)
#file = open(r"test.txt", "r", closefd=True) 

# СЕРИЙНЫЙ НОМЕР ИНВЕРТОРА

input = comm_inverter(QID)
data = input[0]
length = input[1]
crc = input[2]

print(data+crc)
print('length =', length)
file = open(r"test.txt", "a", closefd=True)
f.write(QID + '\n')
f.write(data+crc + '\n')
f.write('length =', length, '\n')

        
# ПАРАМЕТРЫ ИНВЕРТОРА

input = comm_inverter(QPIGS)
data = input[0]
length = input[1]
crc = input[2]

# РЕЖИМ РАБОТЫ ИНВЕРТОРА

input = comm_inverter(QMOD)
data = input[0]
length = input[1]
crc = input[2]


# СОСТОЯНИЕ ИНВЕРТОРА

input = comm_inverter(QPIRI)
data = input[0]
length = input[1]
crc = input[2]

# ОШИБКИ И НЕИСПРАВНОСТИ ИНВЕРТОРА
    

input = comm_inverter(QPIWS)
data = input[0]
length = input[1]
crc = input[2]
