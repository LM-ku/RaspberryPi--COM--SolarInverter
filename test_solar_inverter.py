import serial
import time
 


# КОМАНДЫ + СRC16 ДЛЯ ПЕРЕДАЧИ В ИНВЕРТОР 

QPIGS = [0x51,0x50,0x49,0x47,0x53,0xB7,0xA9,0x0D]                           # Device general status parameters inquiry
QMCHGCR = [0x51,0x4D,0x43,0x48,0x47,0x43,0x52,0xD8,0x55,0x0D]               # Enquiry selectable value about max charging current
QMUCHGCR = [0x51,0x4D,0x55,0x43,0x48,0x47,0x43,0x52,0x26,0x34,0x0D]         # Enquiry selectable value about max utility charging current
QPIWS = [0x51,0x50,0x49,0x57,0x53,0xB4,0xDA,0x0D]                           # Device Warning Status inquiry
QMOD = [0x51,0x4D,0x4F,0x44,0x49,0xC1,0x0D]                                 # Device Mode inquiry
QID = [0x51,0x49,0x44,0xD6,0xEA,0x0D]                                       # The device serial number inquiry
QDI = [0x51,0x44,0x49,0x71,0x1B,0x0D]                                       # The default setting value information
QVFW = [0x51,0x56,0x46,0x57,0x62,0x99,0x0D]                                 # Main CPU Firmware version inquiry
QVFW2 = [0x51,0x56,0x46,0x57,0x32,0xC3,0xF5,0x0D]                           # Another CPU Firmware version inquiry

QPIRI = [0x51,0x50,0x49,0x52,0x49,0xF8,0x54,0x0D]                           # Device Rating Information inquiry

QFLAG = [0x51,0x46,0x4C,0x41,0x47,0x98,0x74,0x0D]                           # Device flag status inquiry 
# ACK = [0x41,0x43,0x4B,0x39,0x20,0x0D]                                     # Device response
POP = {'UTI':[0x50,0x4F,0x50,0x30,0x30,0xC2,0x48,0x0D],                     # Setting device output source priority to UTI
       'SOL':[0x50,0x4F,0x50,0x30,0x31,0xD2,0x69,0x0D],                     # Setting device output source priority to SOL
       'SBU':[0x50,0x4F,0x50,0x30,0x32,0xE2,0x0B,0x0D]                      # Setting device output source priority to SBU
       }
PCP = {'UTI':[0x50,0x43,0x50,0x30,0x30,0x8D,0x7A,0x0D],                     # Setting device charger priority to UTI first
       'SOL':[0x50,0x43,0x50,0x30,0x31,0x9D,0x5B,0x0D],                     # Setting device charger priority to SOL first
       'SOL+UTI':[0x50,0x43,0x50,0x30,0x32,0xAD,0x38,0x0D],                 # Setting device charger priority to SOL+UTI
       'OnlySOL':[0x50,0x43,0x50,0x30,0x33,0xBD,0x19,0x0D]                  # Setting device charger priority to OnlySOL
       }
PGR = {'APP':[0x50,0x47,0x52,0x30,0x30,0x29,0xEB,0x0D],                     # Setting device grid working range to APP
       'UPS':[0x50,0x47,0x52,0x30,0x31,0x39,0xCA,0x0D]                      # Setting device grid working range to UPS
       }                      
PBCV = {'44.0':[0x50,0x42,0x43,0x56,0x34,0x34,0x2E,0x30,0xE6,0xEB,0x0D],    # Set battery re-charge voltage to 44.0 V
        '45.0':[0x50,0x42,0x43,0x56,0x34,0x35,0x2E,0x30,0xD1,0xDB,0x0D],    # Set battery re-charge voltage to 45.0 V
        '46.0':[0x50,0x42,0x43,0x56,0x34,0x36,0x2E,0x30,0x88,0x8B,0x0D],    # Set battery re-charge voltage to 46.0 V
        '47.0':[0x50,0x42,0x43,0x56,0x34,0x37,0x2E,0x30,0xBF,0xBB,0x0D],    # Set battery re-charge voltage to 47.0 V
        '48.0':[0x50,0x42,0x43,0x56,0x34,0x38,0x2E,0x30,0x93,0x8A,0x0D],    # Set battery re-charge voltage to 48.0 V
        '49.0':[0x50,0x42,0x43,0x56,0x34,0x39,0x2E,0x30,0xA4,0xBA,0x0D],    # Set battery re-charge voltage to 49.0 V
        '50.0':[0x50,0x42,0x43,0x56,0x35,0x30,0x2E,0x30,0x4C,0x9F,0x0D],    # Set battery re-charge voltage to 50.0 V
        '51.0':[0x50,0x42,0x43,0x56,0x35,0x31,0x2E,0x30,0x7B,0xAF,0x0D]     # Set battery re-charge voltage to 51.0 V
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
        

set_time = 20.0                                                 # периодичность опроса инвертора xx, s
cmd = ''
info_rd = False
ser_state = False
file_name = r"C:\Users\Сергей\Documents\RaspberryPi\Test_Inverter\test.txt"


# === РАСЧЕТ СRC16 по алгоритму CRC-CCITT (XModem) ===

def crc16(message):
    
# https://bytes.com/topic/python/insights/887357-python-check-crc-frame-crc-16-ccitt      
# CRC-16-CITT poly, the CRC sheme used by ymodem protocol
# 16bit operation register, initialized to zeros
# message - строка данных, для которой расчитывается CRC
# функция возвращает два байта СRC16 вида: b'\x00\x00'

    poly = 0x1021
    reg = 0
    message += b'\x00\x00' 
    for byte in message:
        mask = 0x80
        while(mask > 0):
            reg<<=1
            if byte & mask:     
                reg += 1
            mask>>=1
            if reg > 0xffff:            
                reg &= 0xffff           
                reg ^= poly
    crc_h = reg >> 8
    crc_l = reg & 0xFF                
    return bytes([crc_h, crc_l])

# ======== ОБМЕН С ИНВЕРТОРОМ ЧЕРЕЗ COM-ПОРТ =========

def comm_inverter(msg_wr):

# msg_wr - массив байт для передачи инвертору через СОМ-порт
# length - количество байт, принятых от инвертора через СОМ-порт
# data - строка данных, принятых от инвертора через СОМ-порт 
# rd_crc - 2 байта CRC16, принятых от инвертора через СОМ-порт
# calc_crc - 2 байта CRC16, расчитанной для принятых данных
# сrc_ok - критерий достоверности данных
# функция возвращает кортеж вида : <data> , <length>, <сrc_ok>
# если прием с ошибкой, данные обнуляются


# - инициализация СОМ-порта

    serial_state = False
    сrc_ok = False
    data = ''
    length = 0
    
    try:
        print('инициализация СОМ-порта')
        ser = serial.Serial(
            port='COM3',
            baudrate = 2400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )
    except serial.SerialException:
        print('СОМ-порт не обнаружен\n\r')
        data = ''
        length = 0
        сrc_ok = False
    else:
        print('СОМ-порт ОК\n\r')
        ser.flushInput()

# - передача данных в СОМ-порт

        ser.write(bytearray(msg_wr))
        print('передача запроса в СОМ-порт')

        # --------- записать в файл ---------   
        file.write(bytearray(msg_wr))    
        # -----------------------------------

        print('Write to INVERTER', bytes(msg_wr)) 
        time.sleep(0.5)
    
# - прием данных из СОМ-порта

        msg_rd = ser.readline()
        print('\n\rприем данных из СОМ-порта')

        # --------- записать в файл ---------   
        file.write(msg_rd)
        # -----------------------------------
        
        length = len(msg_rd)
        data = msg_rd[:-3]
        rd_crc = msg_rd[-3:-1]
        calc_crc = crc16(data)
        print('Read from INVERTER', length, 'byte :', msg_rd)
        сrc_ok = rd_crc == calc_crc
        if сrc_ok:
            print('-- Data OK, CRC OK --')
            data = data.decode()

            # --------- записать в файл ---------
            file.write(bytes('-- Data OK, CRC OK --\n\r','utf-8'))
            # -----------------------------------

        else:        
            print('----- CRC error -----')
            data = ''    

            # --------- записать в файл ---------           
            file.write(bytes('----- CRC error -----\n\r','utf-8'))
            # -----------------------------------


        # --------- пропуск строки ----------
        #file.write(bytes('\n\r','utf-8'))
        # -----------------------------------

    finally:
        return data, length, сrc_ok

# ОПРОС ИНВЕРТОРА

time_sta = time.time() - set_time

while True :
    
    if (time.time() - time_sta) >= set_time :
        time_sta = time.time()

        from datetime import datetime
        now = datetime.now()

# СЕРИЙНЫЙ НОМЕР ИНВЕРТОРА

        file = open(file_name, "ab", closefd=True)
        from datetime import datetime
        now = datetime.now()
        print(now.strftime('%d/%m/%y %I:%M:%S'))
        input = comm_inverter(QID)
        data = input[0]
        length = input[1]
        crc_ok = input[2]
        print('\n\rQID :', data, '\nLength =', length, ',', crc_ok, '\n\r')
        file.close()
        #time.sleep(1.0)

# РЕЖИМ РАБОТЫ ИНВЕРТОРА

        file = open(file_name, "ab", closefd=True)
        from datetime import datetime
        now = datetime.now()
        print(now.strftime('%d/%m/%y %I:%M:%S'))
        input = comm_inverter(QMOD)
        data = input[0]
        length = input[1]
        crc_ok = input[2]
        print('\n\rQMOD :', data, '\nLength =', length, ',', crc_ok, '\n\r')
        file.close()
        #time.sleep(1.0)

# ПАРАМЕТРЫ ИНВЕРТОРА

        file = open(file_name, "ab", closefd=True)
        from datetime import datetime
        now = datetime.now()
        print(now.strftime('%d/%m/%y %I:%M:%S'))
        input = comm_inverter(QPIGS)
        data = input[0]
        length = input[1]
        crc_ok = input[2]
        print('\n\rQPIGS :', data, '\nLength =', length, ',', crc_ok, '\n\r')
        file.close()
        #time.sleep(1.0)

# СОСТОЯНИЕ ИНВЕРТОРА

        file = open(file_name, "ab", closefd=True)
        from datetime import datetime
        now = datetime.now()
        print(now.strftime('%d/%m/%y %I:%M:%S'))
        input = comm_inverter(QPIRI)
        data = input[0]
        length = input[1]
        crc_ok = input[2]
        print('\n\rQPIRI:', data, '\nLength =', length, ',', crc_ok, '\n\r')
        file.close()
        #time.sleep(1.0)

# ОШИБКИ И НЕИСПРАВНОСТИ ИНВЕРТОРА

        file = open(file_name, "ab", closefd=True)
        from datetime import datetime
        now = datetime.now()
        print(now.strftime('%d/%m/%y %I:%M:%S'))
        input = comm_inverter(QPIWS)
        data = input[0]
        length = input[1]
        crc_ok = input[2]
        print('\n\rQPIWS:', data, '\nLength =', length, ',', crc_ok, '\n\r')
        file.close()


