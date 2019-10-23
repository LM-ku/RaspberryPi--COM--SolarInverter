#!/usr/bin/python3

import paho.mqtt.client as mqtt
import serial
import time
 
#broker = 'www.mqtt-dashboard.com'
broker = 'localhost'
topic = 'my_solar'


# КОМАНДЫ + СRC16 ДЛЯ ПЕРЕДАЧИ В ИНВЕРТОР 

QPIGS = [0x51,0x50,0x49,0x47,0x53,0xB7,0xA9,0x0D]                           # Device general status parameters inquiry
QPIGS = [0x51,0x50,0x49,0x47,0x53,0xB7,0xA9,0x0D]
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
ACK = [0x41,0x43,0x4B,0x39,0x20,0x0D]                                       # Device response
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
        

device_serial = 'xxxxxxxxxxxxxx'
cmd = ''
set_time = 30                                                   # периодичность опроса инвертора xx, s
st_read = 0
info_rd = False
ser_state = False
mqtt_connect = False


# ================== РАСЧЕТ СRC16 ====================
# процедура расчета CRC16 по алгоритму CRC-CCITT (XModem)
# https://bytes.com/topic/python/insights/887357-python-check-crc-frame-crc-16-ccitt      
# CRC-16-CITT poly, the CRC sheme used by ymodem protocol
# 16bit operation register, initialized to zeros
# message - строка данных, для которой расчитывается CRC
# функция возвращает два байта СRC16

def crc16(message):
    poly = 0x1021
    reg = 0
    message += b'\x00\x00'
    #msg = message.encode("utf-8")
    msg = message
    for byte in msg:
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
# инициализация СОМ-порта, передача запроса к инвертору и получение данных от инвертора
# msg_wr - массив байт для передачи инвертору через СОМ-порт
# length - количество байт, принятых от инвертора через СОМ-порт
# data - строка данных, принятых от инвертора через СОМ-порт 
# rd_crc - 2 байта CRC16, принятых от инвертора через СОМ-порт
# calc_crc - 2 байта CRC16, расчитанной для принятых данных
# сrc_ok - критерий достоверности данных
# функция возвращает кортеж вида : <data> , <length>, <сrc_ok>
# если при инициализации СОМ-порта произошла ошибка или
# данные от инвертора приняты с ошибкой, 
# выходные данные функции обнуляются

def comm_inverter(msg_wr):
    
# - инициализация переменных
    global ser_state
    ser_state = False
    сrc_ok = False
    data = ''
    length = 0
    
# - инициализация СОМ-порта    
    try:
        #print('инициализация СОМ-порта')
# --- для тестирования скрипта на ПК ---
#        ser = serial.Serial(
#            port='COM3',         
#            baudrate = 2400,
#            parity=serial.PARITY_NONE,
#            stopbits=serial.STOPBITS_ONE,
#            bytesize=serial.EIGHTBITS,
#            timeout=1
#        )
# --------------------------------------

        ser = serial.Serial(              
            port='/dev/ttyAMA0',
            baudrate = 2400,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            bytesize=serial.EIGHTBITS,
            timeout=1
        )

# - обработка ошибки инициализации COM-порта

    except serial.SerialException:  
        print('СОМ-порт не обнаружен\n\r')
        data = ''
        length = 0
        сrc_ok = False

# - успешная инициализация - работа с COM-портом
      
    else: 
        ser_state = True
        #print('СОМ-порт ОК\n\r')
#        ser.flushInput()  # очистить буфер ввода
#        ser.flushOutput() # очистить буфер вывода
#        time.sleep(0.1)

# - передача данных в СОМ-порт

        ser.write(bytearray(msg_wr))  
        time.sleep(0.5)
    
# - прием данных из СОМ-порта и вычисление CRC
  
        msg_rd = ser.readline()
        #print('\n\rприем данных из СОМ-порта')
        length = len(msg_rd)
        #data = msg_rd[:-3].decode('utf-8')
        data = msg_rd[:-3]
        rd_crc = msg_rd[-3:-1]
        calc_crc = crc16(data)
        print('Write to INVERTER', bytes(msg_wr))         
        print('Read from INVERTER', length, 'byte :', msg_rd)
        сrc_ok = rd_crc == calc_crc
        if сrc_ok:
            #print('-- Data OK, CRC OK --')
            data = data.decode()
        else:        
            print('----- CRC error -----')
            data = ''    

# - выходные переменные процедуры обмена с инвертором
 
    finally:
        return data, length, сrc_ok
    pass
    
# == ПОДКЛЮЧЕНИЕ К MQTT-БРОКЕРУ И ПОДПИСКА НА ТОПИКИ ==
# client - выходная переменная идентификации клиента
# userdata - выходная переменная ...
# flags - выходная переменная ...
# rc - выходная переменная - код подключения:
#      0: Connection successful 
#      1: Connection refused - incorrect protocol version 
#      2: Connection refused - invalid client identifier 
#      3: Connection refused - server unavailable 
#      4: Connection refused - bad username or password 
#      5: Connection refused - not authorised 
#      6-255: Currently unused.

def on_connect(client, userdata, flags, rc):
    global mqtt_connect
    mqtt_connect = True
    print('Connected...', 'CLIENT:', client, 'USERDATA:', userdata, 'FLAGS:', flags, 'CODE =', rc)
    client.subscribe(topic+'/#')
    
    

def on_disconnect(client, userdata, rc):
    global mqtt_connect
    mqtt_connect = False
    print('Disconnect ', 'CLIENT:', client, 'CODE = ', rc)
   
    



# == ПОЛУЧЕНИЕ КОМАНД ОТ MQTT-БРОКЕРА И ПЕРЕДАЧА ИНВЕРТОРУ ==
# client - выходная переменная идентификации клиента
# userdata - выходная переменная ...
# msg - полученное сообщение:
#        msg.topic - подписка
#        msg.payload - тело сообщения
#        msg.qos - "качество обслуживания"
# при получении от MQTT-брокера сообщения из топика, на который оформлена подписка
# полученное сообщение интерпретируется, как команда для передачи инвертору 
# команда передается инвертору через СОМ-порт
# полученное от инвертора подтверждение от инвертора публикуется на MQTT-брокере 

def on_message(client, userdata, msg):
    #print('Received message...', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', msg.payload, 'QoS =', msg.qos)
    cmd = msg.payload.decode()    
            
# - период опроса инвертора, s

    if msg.topic == topic+'/set/period_s':                              
            set_time = int(cmd)

# передача команд к инвертеру возможна только в том случае,   
# если информация от инвертора ранее уже была получена

    if info_rd :

# - SET device source range

        if msg.topic == topic+'/set/device_source_range' and cmd != source_range :
            #print('SET_source_range :', cmd)
            try:
                reply = comm_inverter(PGR[cmd])                       # 'PGR'+<cmd>+<crc16>
                data = reply[0]
                length = reply[1]
                crc_ok = reply[2]
                if data == '(ACK' and crc_ok : 
                    set_source_range = True
                    #print('SETTTING device source range OK')
                else :
                    set_source_range = False
                    #print('SETTTING device source range ERR')
            except:
                set_source_range = False
                #print('SETTTING device source range CMD WRONG')
            finally:   
                client.publish(topic+'/ack/device_source_range', set_source_range, 0)
                             
# - SET source_priority

        if msg.topic == topic+'/set/source_priority' and cmd != source_priority :
            #print('Set_source_priority :', cmd)
            try:
                reply = comm_inverter(POP[cmd])                       # 'POP'+<cmd>+<crc16>
                data = reply[0]
                length = reply[1]
                crc_ok = reply[2]
                if data == '(ACK' and crc_ok :
                    set_source_priority = True
                    #print('SETTTING source priority OK')
                else :
                    set_source_priority = False
                    #print('SETTTING source priority ERR')
            except:
                set_source_priority = False
                #print('SETTTING source priority CMD WRONG')        
            finally:   
                client.publish(topic+'/ack/source_priority', set_source_priority, 0)
                                
# - SET charger priority

        if msg.topic == topic+'/set/charger_priority' and cmd != charger_priority :
            #print('SET_charger_priority :', cmd)
            try:
                reply = comm_inverter(PCP[cmd])                       # 'PCP'+<cmd>+<crc16>
                data = reply[0]
                length = reply[1]
                crc_ok = reply[2]
                if data == '(ACK' and crc_ok :
                    set_charger_priority = True
                    #print('SETTTING charger priority OK')
                else :
                    set_charger_priority = False
                    #print('SETTTING charger priority ERR')
            except:
                set_charger_priority = False
                #print('SETTTING charger priority CMD WRONG')   
            finally:   
                client.publish(topic+'/ack/charger_priority', set_charger_priority, 0)
                
# - SET batt recharge voltage

        if msg.topic == topic+'/set/batt_recharge_voltage' :
            value = '{:0>4.1f}'.format(float(cmd))  # интерпретировать <cmd>, как число в формате ХХ.Х
            if batt_recharge_voltage != value :
                #print('SET_batt_recharge_voltage :', cmd)
                try:
                    reply = comm_inverter(PBCV[value])                # 'PBCV'+<value>+<crc16>
                    data = reply[0]
                    length = reply[1]
                    crc_ok = reply[2]
                    if data == '(ACK' and crc_ok :
                        set_batt_recharge_voltage = True
                        #print('SETTTING batt recharge voltage OK')
                    else :
                        set_batt_recharge_voltage = False
                        #print('SETTTING batt recharge voltage ERR')
                except:
                    set_batt_recharge_voltage = False
                    #print('SET batt_recharge_voltage VAL = WRONG')
                finally:   
                    client.publish(topic+'/ack/batt_recharge_voltage', set_batt_recharge_voltage, 0)

# - SET batt under voltage

        if msg.topic == topic+'/set/batt_under_voltage' :
            value = '{:0>4.1f}'.format(float(cmd))  # интерпретировать <cmd>, как число в формате ХХ.Х  
            if batt_under_voltage !=  value :
                #print('SET_batt_under_voltage :', cmd) 
                try:
                    reply = comm_inverter(PSDV[value])            # 'PSDV'+<value>+<crc16>
                    data = reply[0]
                    length = reply[1]
                    crc_ok = reply[2]
                    if data == '(ACK' and crc_ok :
                        set_batt_under_voltage = True
                        #print('SETTTING batt under voltage OK')
                    else :
                        set_batt_under_voltage = False
                        #print('SETTTING batt under voltage ERR')
                except:
                    set_batt_under_voltage = False
                    #print('SET batt under voltage VAL WRONG') 
                finally:   
                    client.publish(topic+'/ack/batt_under_voltage', set_batt_under_voltage, 0)   
                 
# - SET batt redischarge voltage

        if msg.topic == topic+'/set/batt_redischarge_voltage' :
            value = '{:0>4.1f}'.format(float(cmd))  # интерпретировать <cmd>, как число в формате ХХ.Х    
            if batt_redischarge_voltage !=  value :
                #print('SET_batt_redischarge_voltage :', value) 
                try:
                    reply = comm_inverter(PBDV[value])        # 'PBDV'+<value>+<crc16>
                    data = reply[0]
                    length = reply[1]
                    crc_ok = reply[2]
                    if data == '(ACK' and crc_ok :
                        set_batt_redischarge_voltage = True
                        #print('SETTTING batt redischarge voltage OK')
                    else :
                        set_batt_redischarge_voltage = False
                        #print('SETTTING batt redischarge voltage ERR')
                except:
                    set_batt_redischarge_voltage = False
                    #print('SET batt_redischarge voltage VAL WRONG') 
                finally:   
                    client.publish(topic+'/ack/batt_redischarge_voltage', set_batt_redischarge_voltage, 0)   
    pass 

# ===== ПОДТВЕРЖДЕНИЕ ПУБЛИКАЦИИ НА MQTT-БРОКЕРЕ =====

def on_publish(client, userdata, mid):
    #print('Publish OK...', 'CLIENT:', client, 'USERDATA:', userdata, 'MID =', mid)
    pass



# КЛИЕНТ MQTT

# - Client ID = серийный номер инвертора 

while device_serial == 'xxxxxxxxxxxxxx' and st_read < 3:
    st_read = st_read + 1
    reply = comm_inverter(QID)
    data = reply[0]
    length = reply[1]
    crc_ok = reply[2]
    if crc_ok and length == 26 : device_serial = data[1:]
    print(st_read, ':', device_serial)
st_read = 0 


client = mqtt.Client(device_serial)
try: 
    client.connect(broker, 1883, 60)
except:
    #print('MQTT брокер не найден !')
    pass
client.on_connect = on_connect
client.on_disconnect = on_disconnect
client.on_message = on_message
#client.on_publish = on_publish
client.loop_start()



# ПЕРИОДИЧЕСКИЙ ОПРОС ИНВЕРТОРА И ПУБЛИКАЦИЯ ДАННЫХ У БРОКЕРА
# publish(topic, payload, wait_for_publish)
# wait_for_publish == 0 - публикация вне зависимости от наличия связи с брокером, данные могут быть потеряны
# wait_for_publish == 1 - публикация состоится только при наличии связи с брокером, все данные будут опубликованы после соединения с брокером

time_pre = time.time() - set_time
ac_energy_pre = 0                                   # потребление энергии x.хххх, kW*h 
pv_energy_pre = 0.0                                 # энергия солнечных панелей х.хxxx, kW*h

while True :

    cycle = time.time() - time_pre
    if cycle >= set_time :
        time_pre = time.time()
        if mqtt_connect :
            #client.publish(topic+'/script_state', True, 0) 
        
# - параметры инвертора

            grid_voltage = 0.0                              # напряжение сети xxx.x, V
            grid_frequency = 0.0                            # частота сети хх.х, Hz
            ac_voltage = 0.0                                # выходное напряжение инвертора ххх.х, V
            ac_frequency = 0.0                              # частота на выходе инвертора хх.х, Hz
            ac_va_power = 0                                 # полная выходная мощность хххх, VA
            ac_w_power = 0                                  # активная выходная мощность хххх, W
            #ac_energy = 0                                   # потребление энергии x.хххх, kW*h 
            ac_load = 0                                     # нагрузка инвертора ххх, %
            bus_voltage = 0                                 # напряжение шины постоянного тока ххх, V
            batt_voltage = 0.0                              # напряжение батареи хх.х, V 
            batt_charging = 0.0                             # ток заряда батареи хх.х, А 
            batt_capacity = 0                               # емкость батареи xxx, %
            temp_inverter = 0                               # температура инвертора xxxx, T
            pv_current = 0                                  # выходной ток солнечных панелей xxxx, A
            pv_voltage = 0.0                                # выходное напряжение солнечных панелей xxx.x, V
            pv_power = 0.0                                  # выходная мощность солнечных панелей xxxx.х, W
            #pv_energy = 0.0                                 # энергия солнечных панелей х.хxxx, kW*h 
            scc_voltage = 0.0                               # напряжение заряда от солнечных панелей xx.xx, V
            batt_discharge = 0                              # разрядный ток от аккумулятора ххх, А
            device_status = '00000000'                      # байт состояния инвертора
            load = 0                                        # нагрузка инвртора
            charging = 'unknow'                             # источник зарядки батареи
            charging_status = 0                             # Charging On/Off
            charging_scc = 0                                # Charging with SCC
            charging_grid = 0                               # Charging with AC grid
            comm_state = 'ok'                               # состояние обмена с инвертором
        
            reply = comm_inverter(QPIGS)
            data = reply[0]
            length = reply[1]
            crc_ok = reply[2]
            print(data, 'len =', length) 
            if crc_ok and length == 110 :                 
                try:
                    # для корректного представления в числовом виде на всякий случай предусмотрена замена ',' на '.'
                    grid_voltage = float(data[1:6].replace(',', '.'))
                    grid_frequency = float(data[7:11].replace(',', '.'))
                    ac_voltage = float(data[12:17].replace(',', '.'))
                    ac_frequency = float(data[18:22].replace(',', '.'))
                    ac_va_power = int(data[23:27])
                    ac_w_power = int(data[28:32])
                    ac_load = int(data[33:36])
                    bus_voltage = int(data[37:40])
                    batt_voltage = float(data[41:46].replace(',', '.'))
                    batt_charging = float(data[47:50].replace(',', '.'))/10.0
                    batt_capacity = int(data[51:54])
                    temp_inverter = int(data[55:59])
                    pv_current = int(data[60:64])
                    pv_voltage = float(data[65:70].replace(',', '.'))
                    scc_voltage = float(data[71:76].replace(',', '.'))
                    batt_discharge = int(data[77:82])
                    device_status = data[83:91]                    
                    load = device_status[3]
                    charging_status = device_status[5]
                    charging_scc = device_status[6]
                    charging_grid = device_status[7]
                    
                    if device_status[5:] == '000' : charging = 'NOT charging'
                    if device_status[5:] == '110' : charging = 'Charging with SCC'
                    if device_status[5:] == '101' : charging = 'Charging with AC grid'
                    if device_status[5:] == '111' : charging = 'Charging with SCC + AC grid'
                except: 
                    comm_state = 'decryption error'
            else:            
                if ser_state : comm_state = 'read error'
                else : comm_state = 'COM-port error'

            ac_energy = round(((ac_energy_pre + float(ac_w_power * cycle / 3600000.0)) / 2), 4) # расчет потребления энергии за период опроса инвертора
            ac_energy_pre = float(ac_w_power * cycle / 3600000.0)
            pv_power = round(float(pv_current * pv_voltage),2)
            pv_energy = round(((pv_energy_pre + float(pv_power * cycle / 3600000.0)) / 2), 4)   # расчет солнечной энергии за период опроса инвертора 
            pv_energy_pre = float(pv_power * cycle / 3600000.0)
   
            print('QPIGS : ', comm_state)
    
            client.publish(topic+'/status/grid_voltage', grid_voltage, 0)
            client.publish(topic+'/status/grid_frequency', grid_frequency, 0)
            client.publish(topic+'/status/ac_voltage', ac_voltage, 0)
            client.publish(topic+'/status/ac_frequency', ac_frequency, 0)
            client.publish(topic+'/status/ac_va_power', ac_va_power, 0)
            client.publish(topic+'/status/ac_w_power', ac_w_power, 0)
            client.publish(topic+'/status/ac_energy', ac_energy, 0)
            client.publish(topic+'/status/ac_load', ac_load, 0)
            client.publish(topic+'/status/bus_voltage', bus_voltage, 0)
            client.publish(topic+'/status/batt_voltage', batt_voltage, 0)
            client.publish(topic+'/status/batt_charging', batt_charging, 0)
            client.publish(topic+'/status/batt_capacity', batt_capacity, 0)
            client.publish(topic+'/status/temp_inverter', temp_inverter, 0)
            client.publish(topic+'/status/pv_current', pv_current, 0)
            client.publish(topic+'/status/pv_voltage', pv_voltage, 0)
            client.publish(topic+'/status/pv_power', pv_power, 0)
            client.publish(topic+'/status/pv_energy', pv_energy, 0)
            client.publish(topic+'/status/scc_voltage', scc_voltage, 0)
            client.publish(topic+'/status/batt_discharge', batt_discharge, 0)
            client.publish(topic+'/status/load', load, 0)
            client.publish(topic+'/status/charging', charging, 0)
            client.publish(topic+'/status/charging_status', charging_status, 0)
            client.publish(topic+'/status/charging_scc', charging_scc, 0)
            client.publish(topic+'/status/charging_grid', charging_grid, 0)
            client.publish(topic+'/status/QPIGS_comm', comm_state, 0)    
     

# - режим работы инвертора

            mode = 'unknow'
            comm_state = 'ok'  

            reply = comm_inverter(QMOD)
            data = reply[0]
            length = reply[1]
            crc_ok = reply[2]
            print(data, 'len =', length) 
            if crc_ok and length == 5 :
                try: 
                    if data == '(P' : mode = 'Pover On mode'
                    if data == '(S' : mode = 'Standby mode'
                    if data == '(L' : mode = 'Line mode'
                    if data == '(B' : mode = 'Battery mode'
                    if data == '(F' : mode = 'Fault mode'
                    if data == '(H' : mode = 'Pover saving mode'    
                except: 
                    comm_state = 'decryption error'
            else:
                if ser_state : comm_state = 'read error'
                else : comm_state = 'COM-port error'

            print('QMOD : ', comm_state)
            
            client.publish(topic+'/mode/mode', mode , 0)
            client.publish(topic+'/mode/QMOD_comm', comm_state , 0)

# - состояние инвертора

            source_range = 'unknow'
            source_priority = 'unknow'
            charger_priority = 'unknow'
            batt_recharge_voltage = 0.0
            batt_under_voltage = 0.0
            batt_redischarge_voltage = 0.0
            comm_state = 'ok'
         
            reply = comm_inverter(QPIRI)
            data = reply[0]
            length = reply[1]
            crc_ok = reply[2]
            print(data, 'len =', length)
            if crc_ok and length == 98 :
                try:
                    if data[72] == '0' : source_range = 'APP'
                    if data[72] == '1' : source_range = 'UPS' 
        
                    if data[74] == '0' : source_priority = 'UTI'
                    if data[74] == '1' : source_priority = 'SOL'
                    if data[74] == '2' : source_priority = 'SBU'     
        
                    if data[76] == '0' : charger_priority = 'UTI'
                    if data[76] == '1' : charger_priority = 'SOL'
                    if data[76] == '2' : charger_priority = 'SOL+UTI'
                    if data[76] == '3' : charger_priority = 'OnlySOL'       
       
                
                    batt_recharge_voltage = float(data[43:47].replace(',', '.'))
                    batt_under_voltage = float(data[48:52].replace(',', '.'))
                    batt_redischarge_voltage = float(data[87:91].replace(',', '.'))
                    info_rd = True
                except: 
                    comm_state = 'decryption error'             
            else:
                if ser_state : comm_state = 'read error'
                else : comm_state = 'COM-port error'
   
            print('QPIRI : ', comm_state)

            client.publish(topic+'/info/source_range', source_range , 0)
            client.publish(topic+'/info/source_priority', source_priority , 0)
            client.publish(topic+'/info/charger_priority', charger_priority , 0)
            client.publish(topic+'/info/batt_recharge_voltage', batt_recharge_voltage , 0)
            client.publish(topic+'/info/batt_under_voltage', batt_under_voltage , 0)
            client.publish(topic+'/info/batt_redischarge_voltage', batt_redischarge_voltage , 0)
            client.publish(topic+'/info/QPIRI_comm', comm_state, 0)
         

# - ошибки и неисправности инвертора
    
            alarm_1 = 'none'
            alarm_2 = 'none'
            alarm_3 = 'none'
            alarm_4 = 'none'
            alarm_5 = 'none'
            alarm_6 = 'none'
            alarm_7 = 'none'
            alarm_8 = 'none'
            alarm_9 = 'none'
            alarm_10 = 'none'
            alarm_11 = 'none'
            alarm_12 = 'none'
            alarm_13 = 'none'
            alarm_14 = 'none'
            alarm_15 = 'none'
            alarm_16 = 'none'
            alarm_17 = 'none'
            alarm_18 = 'none'
            alarm_19 = 'none'
            alarm_20 = 'none'
            alarm_21 = 'none'
            alarm_22 = 'none'
            alarm_23 = 'none'
            alarm_24 = 'none'
            alarm_25 = 'none'
            alarm_26 = 'none'
            alarm_27 = 'none'
            alarm_28 = 'none'
            alarm_29 = 'none'
            alarm_30 = 'none'
            alarm_31 = 'none'
            alarm_32 = 'none'  
            comm_state = 'ok'
        
            reply = comm_inverter(QPIWS)
            data = reply[0]
            length = reply[1]
            crc_ok = reply[2]
            print(data, 'len =', length)
            if crc_ok and length == 36 :
                alarm = 'Warning'
                try:
                    if data[1] == '1' : alarm_1 = 'Fault'
                    if data[2] == '1' : alarm_2 = 'Fault'; alarm = 'Fault'
                    if data[3] == '1' : alarm_3 = 'Fault'
                    if data[4] == '1' : alarm_4 = 'Fault'
                    if data[5] == '1' : alarm_5 = 'Fault'
                    if data[6] == '1' : alarm_6 = 'Fault'
                    if data[7] == '1' : alarm_7 = 'Fault'
                    if data[8] == '1' : alarm_8 = 'Fault'
                    if data[9] == '1' : alarm_9 = 'Fault'
                    if data[10] == '1' : alarm_10 = alarm
                    if data[11] == '1' : alarm_11 = alarm
                    if data[12] == '1' : alarm_12 = alarm 
                    if data[13] == '1' : alarm_13 = 'Warning'
                    if data[14] == '1' : alarm_14 = 'Fault'
                    if data[15] == '1' : alarm_15 = 'Warning'
                    if data[16] == '1' : alarm_16 = 'Fault'
                    if data[17] == '1' : alarm_17 = alarm
                    if data[18] == '1' : alarm_18 = 'Fault'
                    if data[19] == '1' : alarm_19 = 'Fault'
                    if data[20] == '1' : alarm_20 = 'Fault'
                    if data[21] == '1' : alarm_21 = 'Fault'
                    if data[22] == '1' : alarm_22 = 'Fault'
                    if data[23] == '1' : alarm_23 = 'Fault'
                    if data[24] == '1' : alarm_24 = 'Fault'
                    if data[25] == '1' : alarm_25 = 'Fault'
                    if data[26] == '1' : alarm_26 = 'Warning'
                    if data[27] == '1' : alarm_27 = 'Warning'
            
                    if data[29] == '1' : alarm_28 = 'Warning'
                    if data[28] == '1' : alarm_28 = 'Fault'
            
                    if data[30] == '1' : alarm_30 = 'Warning'
                    if data[31] == '1' : alarm_31 = 'Fault'
                    if data[32] == '1' : alarm_32 = 'Fault'  
                except: 
                    comm_state = 'decryption error' 
            else:
                if ser_state : comm_state = 'read error'
                else : comm_state = 'COM-port error'

            print('QPIWS : ', comm_state) 

            #client.publish(topic+'/alarm/alarm_1', alarm_1, 0)
            client.publish(topic+'/alarm/inverter', alarm_2, 0)
            client.publish(topic+'/alarm/bus_over', alarm_3, 0)
            client.publish(topic+'/alarm/bus_under', alarm_4, 0)
            client.publish(topic+'/alarm/bus_soft_fail', alarm_5, 0)
            client.publish(topic+'/alarm/line_fail', alarm_6, 0)
            client.publish(topic+'/alarm/opv_short', alarm_7, 0)
            client.publish(topic+'/alarm/inverter_voltage_too_low', alarm_8, 0)
            client.publish(topic+'/alarm/inverter_voltage_too_high', alarm_9, 0)
            client.publish(topic+'/alarm/over_temperature', alarm_10, 0)
            client.publish(topic+'/alarm/fan_locked', alarm_11, 0)
            client.publish(topic+'/alarm/battery_voltage_high', alarm_12, 0) 
            client.publish(topic+'/alarm/battery_low', alarm_13, 0)
            #client.publish(topic+'/alarm/alarm_14', alarm_14, 0)
            client.publish(topic+'/alarm/battery_under_shutdown', alarm_15, 0)
            #client.publish(topic+'/alarm/alarm_16', alarm_16, 0)
            client.publish(topic+'/alarm/over_load', alarm_17, 0)
            client.publish(topic+'/alarm/eeprom fault', alarm_18, 0)
            client.publish(topic+'/alarm/inverter_over_current', alarm_19, 0)
            client.publish(topic+'/alarm/inverter_soft_fail', alarm_20, 0)
            client.publish(topic+'/alarm/self_test_fail', alarm_21, 0)
            client.publish(topic+'/alarm/op_dc_voltage_over', alarm_22, 0) 
            client.publish(topic+'/alarm/bat_open', alarm_23, 0)
            client.publish(topic+'/alarm/current_sensor_fail', alarm_24, 0)
            client.publish(topic+'/alarm/battery_short', alarm_25, 0)
            client.publish(topic+'/alarm/power_limit', alarm_26, 0)
            client.publish(topic+'/alarm/pv_voltage_high', alarm_27, 0)
            client.publish(topic+'/alarm/mppt_overload', alarm_28, 0)
        
            client.publish(topic+'/alarm/battery_too_low_to_charge', alarm_30, 0)
            #client.publish(topic+'/alarm/alarm_31', alarm_31, 0)
            #client.publish(topic+'/alarm/alarm_32', alarm_32, 0)  
            client.publish(topic+'/alarm/QPIWS_comm', comm_state, 0)
