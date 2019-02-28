import paho.mqtt.client as mqtt
import serial
import time
 
#broker = 'www.mqtt-dashboard.com'
broker = 'localhost'
topic = 'my_solar'

# ОПРЕДЕЛЕНИЕ СОМ-ПОРТА ДЛЯ ОБМЕНА С ИНВЕРТОРОМ

ser = serial.Serial(              
    port='/dev/ttyAMA0',
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
        
mode = {'(P':'Pover On mode',
        '(S':'Standby mode',
        '(L':'Line mode',
        '(B':'Battery mode',
        '(F':'Fault mode',
        '(H':'Pover saving mode'
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

    ser.write(bytes(msg_wr, 'utf-8'))
    time.sleep(0.5)
    msg_rd = ser.readline().decode()
    data = msg_rd[:-3]
    length = len(data)
    crc = msg_rd[-3:-1]
#    if crc16(data) != crc : length = 0
    return data, length, crc


def on_connect(client, userdata, flags, rc):

# ПОДКЛЮЧЕНИЕ К MQTT-БРОКЕРУ И ПОДПИСКА НА ТОПИКИ
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

    #print('Connected...', 'CLIENT:', client, 'USERDATA:', userdata, 'FLAGS:', flags, 'CODE =', rc)
    client.subscribe(topic+'/#')


def on_message(client, userdata, msg):

# ПОЛУЧЕНИЕ КОМАНД ОТ MQTT-БРОКЕРА И ПЕРЕДАЧА ИНВЕРТОРУ
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

    #print('Received message...', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', msg.payload, 'QoS =', msg.qos)
    # value = str(msg.payload)[2:-1]
    value = msg.payload.decode()
    set_value = '{:0>4.1f}'.format(float(value))
    if msg.topic == topic+'/set/period_s':                              # период опроса инвертора, s
            #print('During :', value)
            set_time = int(value)  
       
    if info_rd :                                                        # если информация от инвертора уже была получена
        if [msg.topic == topic+'/set/device_source_range' and           # 'PGR'+<value>+<crc16>
            ((value == 'APP' and source_range != 'APP') or
             (value == 'UPS' and source_range != 'UPS'))] :             
            #print('SET_source_range :', value)
            ack = comm_inverter(PGR[value])
            if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                set_source_range = True
                #print('SETTTING device_source_range OK')
            else :
                set_source_range = False
                #print('SETTTING device_source_range ERR')
            client.publish(topic+'/ack/device_source_range', set_source_range, 0)
                             
        elif [msg.topic == topic+'/set/source_priority' and             # 'POP'+<value>+<crc16>
              ((value == 'UTI' and source_priority != 'UTI') or
               (value == 'SOL' and source_priority != 'SOL') or
               (value == 'SBU' and source_priority != 'SBU'))] :
            #print('Set_source_priority :', value)
            ack = comm_inverter(POP[value])
            if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                set_source_priority = True
                #print('SETTTING source_priority OK')
            else :
                set_source_priority = False
                #print('SETTTING source_priority ERR')
            client.publish(topic+'/ack/source_priority', set_source_range, 0)
                                
        elif [msg.topic == topic+'/set/charger_priority' and              # 'PCP'+<cmd>+<crc16>
              ((value == 'UTI' and charger_priority != 'UTI') or
               (value == 'SOL' and charger_priority != 'SOL') or
               (value == 'SOL+UTI' and charger_priority != 'SOL+UTI') or
               (value == 'OnlySOL' and charger_priority != 'OnlySOL'))] :
            #print('SET_charger_priority :', value)
            ack = comm_inverter(PCP[value])
            if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                set_charger_priority = True
                #print('SETTTING charger_priority OK')
            else :
                set_charger_priority = False
                #print('SETTTING charger_priority ERR')
            client.publish(topic+'/ack/charger_priority', set_charger_priority, 0)
                
        elif [msg.topic == topic+'/set/batt_recharge_voltage' and         # 'PBCV'+<set_value>+<crc16>
              batt_recharge_voltage != set_value] :
            #print('SET_batt_recharge_voltage :', value)
            try:
                ack = comm_inverter(PBCV[set_value])
                if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                    set_batt_recharge_voltage = True
                    #print('SETTTING batt_recharge_voltage OK')
                else :
                    set_batt_recharge_voltage = False
                    #print('SETTTING batt_recharge_voltage ERR')
            except:
                set_batt_recharge_voltage = False
                #print('SET batt_recharge_voltage VAL WRONG')
            client.publish(topic+'/ack/batt_recharge_voltage', set_batt_recharge_voltage, 0)
                    
        elif [msg.topic == topic+'/set/batt_under_voltage' and            # 'PSDV'+<set_value>+<crc16>
            batt_under_voltage != set_value] :
            #print('SET_batt_under_voltage :', value) 
            try:
                ack = comm_inverter(PSDV[set_value])
                if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                    set_batt_under_voltage = True
                    #print('SETTTING batt_under_voltage OK')
                else :
                    set_batt_under_voltage = False
                    #print('SETTTING batt_under_voltage ERR')
            except:
                set_batt_under_voltage = False
                #print('SET batt_under_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_under_voltage', set_batt_under_voltage, 0)   
                 
        elif [msg.topic == topic+'/set/batt_redischarge_voltage' and      # 'PBDV'+<set_value>+<crc16>
            batt_redischarge_voltage != set_value] :
            #print('SET_batt_redischarge_voltage :', value) 
            try:
                ack = comm_inverter(PBDV[set_value])
                if (ack[0] == '(ACK' ) & (ack[2] == '\x39\x20') :
                    set_batt_redischarge_voltage = True
                    #print('SETTTING batt_redischarge_voltage OK')
                else :
                    set_batt_redischarge_voltage = False
                    #print('SETTTING batt_redischarge_voltage ERR')
            except:
                set_batt_redischarge_voltage = False
                #print('SET batt_redischarge_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_redischarge_voltage', set_batt_redischarge_voltage, 0)   
 

# ПОДТВЕРЖДЕНИЕ ПУБЛИКАЦИИ НА MQTT-БРОКЕРЕ

def on_publish(client, userdata, mid):
    # print('Publish OK...', 'CLIENT:', client, 'USERDATA:', userdata, 'MID =', mid)
    return

# СЕРИЙНЫЙ НОМЕР ИНВЕРТОРА  (92931704100529)

device_serial = 'xxxxxxxxxxxxxx'

while device_serial == 'xxxxxxxxxxxxxx' :

    input = comm_inverter(QID)
    data = input[0]
    length = input[1]
    crc = input[2]
    if (crc == crc16(data)) & (length >= 15) : device_serial = data[1:]

# КЛИЕНТ MQTT

client = mqtt.Client(device_serial)
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish
client.connect(broker, 1883, 60)
client.loop_start()	
#client.loop_forever()



# ОПРОС ИНВЕРТОРА И ПЕРЕДАЧА ДАННЫХ БРОКЕРУ (ПУБЛИКАЦИИ)
# publish(topic, payload, wait_for_publish)
# wait_for_publish == 0 - публикация вне зависимости от наличия связи с брокером, данные могут быть потеряны
# wait_for_publish == 1 - публикация состоится только при наличии связи с брокером, все данные будут опубликованы после соединения с брокером

time_sta = time.time() + set_time

while True :

    if (time.time() - time_sta) >= set_time :
        time_sta = time.time()
        
# - режим работы инвертора

        device_mode = 'unknow'
        comm_state = 'ok'  

        input = comm_inverter(QMOD)
        data = input[0]
        length = input[1]
        crc = input[2]
        if (crc == crc16(data)):
            print(data, 'len =', length)
            try:
                device_mode = mode[data]    
            except:
                #print('QMOD : VAL WRONG')
                pass
        else:
            comm_state = 'err'
            print('QMOD : COMM.ERROR')
            
        client.publish(topic+'/mode/mode', device_mode , 0)
        client.publish(topic+'/mode/QMOD_comm', comm_state , 0)


# - параметры инвертора

        grid_voltage = 0.0                              # напряжение сети xxx.x, V
        grid_frequency = 0.0                            # частота сети хх.х, Hz
        ac_voltage = 0.0                                # выходное напряжение инвертора ххх.х, V
        ac_frequency = 0.0                              # частота на выходе инвертора хх.х, Hz
        ac_va_power = 0                                 # полная выходная мощность хххх, VA
        ac_w_power = 0                                  # активная выходная мощность хххх, W
        ac_load = 0                                     # нагрузка инвертора ххх, %
        bus_voltage = 0                                 # напряжение шины постоянного тока ххх, V
        batt_voltage = 0.0                              # напряжение батареи хх.х, V 
        batt_charging = 0.0                             # ток заряда батареи хх.х, А 
        batt_capacity = 0                               # емкость батареи xxx, %
        temp_inverter = 0                               # температура инвертора xxxx, T
        pv_current = 0                                  # выходной ток солнечных панелей xxxx, A
        pv_voltage = 0.0                                # выходное напряжение солнечных панелей xxx.x, V
        pv_power = 0                                    # выходная солнечная мощность xxxx, W
        scc_voltage = 0.0                               # напряжение заряда от солнечных панелей xx.xx, V
        batt_discharge = 0                              # разрядный ток от аккумулятора ххх, А
        device_status = 'unknow'                        # байт состояния инвертора
        load = 'unknow'                                 # нагрузка инвртора
        charging = 'unknow'                             # источник зарядки батареи
        comm_state = 'ok'                               # ошибка обмена с инвертором
        
        input = comm_inverter(QPIGS)
        data = input[0]
        length = input[1]
        crc = input[2]
        if (crc == crc16(data)):
            print(data, 'len =', length)
            grid_voltage = float(data[1:6])
            grid_frequency = float(data[7:11])
            ac_voltage = float(data[12:17])
            ac_frequency = float(data[18:22])
            ac_va_power = int(data[23:27])
            ac_w_power = int(data[28:32])
            ac_load = int(data[33:36])
            bus_voltage = int(data[37:40])
            batt_voltage = float(data[41:46])
            batt_charging = float(data[47:50])/10.0
            batt_capacity = int(data[51:54])
            temp_inverter = int(data[55:59])
            pv_current = int(data[60:64])
            pv_voltage = float(data[65:70])
            pv_power = int(pv_current) * float(pv_voltage)
            scc_voltage = float(data[71:76])
            batt_discharge = int(data[77:82])
            device_status = data[83:91]
    
            if device_status[3] == '1' : load = 'on' 
            if device_status[3] == '0' : load = 'off' 
     
            if device_status[5:] == '000' : charging = 'NOT charging'
            if device_status[5:] == '110' : charging = 'Charging with SCC'
            if device_status[5:] == '101' : charging = 'Charging with AC grid'
            if device_status[5:] == '111' : charging = 'Charging with SCC + AC grid'


        else:
            comm_state = 'err'
            print('QPIGS : COMM.ERROR') 
   
        client.publish(topic+'/status/grid_voltage', grid_voltage, 0)
        client.publish(topic+'/status/grid_frequency', grid_frequency, 0)
        client.publish(topic+'/status/ac_voltage', ac_voltage, 0)
        client.publish(topic+'/status/ac_frequency', ac_frequency, 0)
        client.publish(topic+'/status/ac_va_power', ac_va_power, 0)
        client.publish(topic+'/status/ac_w_power', ac_w_power, 0)
        client.publish(topic+'/status/ac_load', ac_load, 0)
        client.publish(topic+'/status/bus_voltage', bus_voltage, 0)
        client.publish(topic+'/status/batt_voltage', batt_voltage, 0)
        client.publish(topic+'/status/batt_charging', batt_charging, 0)
        client.publish(topic+'/status/batt_capacity', batt_capacity, 0)
        client.publish(topic+'/status/temp_inverter', temp_inverter, 0)
        client.publish(topic+'/status/pv_current', pv_current, 0)
        client.publish(topic+'/status/pv_voltage', pv_voltage, 0)
        client.publish(topic+'/status/pv_power', pv_power, 0)
        client.publish(topic+'/status/scc_voltage', scc_voltage, 0)
        client.publish(topic+'/status/batt_discharge', batt_discharge, 0)
        client.publish(topic+'/status/load', load, 0)
        client.publish(topic+'/status/charging', charging, 0)
        client.publish(topic+'/status/QPIGS_comm', comm_state, 0)    
     


# - состояние инвертора

        source_range = 'unknow'
        source_priority = 'unknow'
        charger_priority = 'unknow'
        batt_recharge_voltage = 0.0
        batt_under_voltage = 0.0
        batt_redischarge_voltage = 0.0
        comm_state = 'ok'
         
        input = comm_inverter(QPIRI)
        data = input[0]
        length = input[1]
        crc = input[2]
        if (crc == crc16(data)):
            print(data, 'len =', length)
            info_rd = True            
            if data[72] == '0' : source_range = 'APP'
            if data[72] == '1' : source_range = 'UPS' 
        
            if data[74] == '0' : source_priority = 'UTI'
            if data[74] == '1' : source_priority = 'SOL'
            if data[74] == '2' : source_priority = 'SBU'     
        
            if data[76] == '0' : charger_priority = 'UTI'
            if data[76] == '1' : charger_priority = 'SOL'
            if data[76] == '2' : charger_priority = 'SOL+UTI'
            if data[76] == '3' : charger_priority = 'OnlySOL'       
       
            batt_recharge_voltage = float(data[43:47])
            batt_under_voltage = float(data[48:52])
            batt_redischarge_voltage = float(data[87:91])
             
        else:
            comm_state = 'err'
            print('QPIRI : COMM.ERROR')

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
        
        input = comm_inverter(QPIWS)
        data = input[0]
        length = input[1]
        crc = input[2]
        if (crc == crc16(data)):
            print(data, 'len =', length) 
            alarm = 'Warning'
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
 
        else:
            comm_state = 'err' 
            print('QPIWS : COMM.ERROR')

#        client.publish(topic+'/alarm/alarm_1', alarm_1, 0)
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
#        client.publish(topic+'/alarm/alarm_14', alarm_14, 0)
        client.publish(topic+'/alarm/battery_under_shutdown', alarm_15, 0)
#        client.publish(topic+'/alarm/alarm_16', alarm_16, 0)
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
#        client.publish(topic+'/alarm/alarm_31', alarm_31, 0)
#        client.publish(topic+'/alarm/alarm_32', alarm_32, 0)  
        client.publish(topic+'/alarm/QPIWS_comm', comm_state, 0)
