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

# ВЫХОДНЫЕ ПАРАМЕТРЫ ИНВЕРТОРА

grid_voltage = 0.0                              # напряжение сети xxx.x, V
grid_frequency = 0.0                            # частота сети хх.х, Hz
ac_voltage = 0.0                                # выходное напряжение инвертора ххх.х, V
ac_frequency = 0.0                              # частота на выходе инвертора хх.х, Hz
ac_va_power = 0                                 # полная выходная мощность хххх, VA
ac_w_power = 0                                  # активная выходная мощность хххх, W
ac_load = 0                                     # нагрузка инвертора ххх, %
bus_voltage = 0                                 # напряжение шины постоянного тока ххх, V
batt_voltage = 0.0 
batt_charging = 0
batt_capacity = 0                               # емкость батареи xxx, %
temp_inverter = 0                               # температура инвертора xxxx, T
pv_current = 0                                  # выходной ток солнечных панелей xxxx, A
pv_voltage                                      # выходное напряжение солнечных панелей xxx.x, V
pv_power                                        # выходная солнечная мощность xxxx, W
scc_voltage                                     # напряжение заряда от солнечных панелей xx.xx, V
batt_discharge                                  # разрядный ток от аккумулятора ххх, А

# ИНФОРМАЦИЯ О СОСТОЯНИИ ИНВЕРТОРА

source_range = ''                               # (APP/UPS)			
source_priority = ''                            # (SOL/UTI/SBU)		
charger_priority = ''                           # (UTIL/SOL/SOL+UTIL/OnlySOL)	
mode = ''                                       # (P,S,L,B,F,H)				
load_on = ''					
charging = ''                                   # (SCC,AC,SCC+AC)		
grid_rating_voltage = '000.0'                   # xxx.x, V
grid_rating_current = '00.0'                    # xx.x, A		
#		AC_rating_current				 xx.x	A		
#		AC_rating_frequency			 	 xx.x	Hz
#		AC_rating active_power	 	 	 xxxx	W
#		BATT_rating_voltage				xx.xx	V
#		BATT_recharge voltage			xx.xx	V	
#		BATT_under_voltage				xx.xx	V		
#		BATT_bulk_voltage				xx.xx	V		
#		BATT_float_voltage				xx.xx	V



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
        '47.0':'\x50\x53\x44\x56\x34\x37\x2E\x30  <crc> \x0D'   # Set battery under voltage to 47.0 V
        '48.0':'\x50\x53\x44\x56\x34\x38\x2E\x30  <crc> \x0D'   # Set battery under voltage to 48.0 V
        }
PBDV = {'48.0':'\x50\x42\x44\x56\x34\x38\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 48.0 V
        '49.0':'\x50\x53\x44\x56\x34\x39\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 49.0 V
        '50.0':'\x50\x53\x44\x56\x35\x30\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 50.0 V
        '51.0':'\x50\x53\x44\x56\x35\x31\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 51.0 V
        '52.0':'\x50\x53\x44\x56\x35\x32\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 52.0 V
        '53.0':'\x50\x53\x44\x56\x35\x33\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 53.0 V
        '54.0':'\x50\x53\x44\x56\x35\x34\x2E\x30  <crc> \x0D',  # Set battery re-discharge voltage to 54.0 V
        '55.0':'\x50\x53\x44\x56\x35\x35\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 55.0 V
        '56.0':'\x50\x53\x44\x56\x35\x36\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 56.0 V
        '57.0':'\x50\x53\x44\x56\x35\x37\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 57.0 V
        '58.0':'\x50\x53\x44\x56\x35\x38\x2E\x30  <crc> \x0D'   # Set battery re-discharge voltage to 58.0 V        
        }
        

set_time = 10                                                   # периодичность опроса инвертора xx, s
cmd = ''
status_rd = False

# ОБМЕН С ИНВЕРТОРОМ ЧЕРЕЗ COM-ПОРТ
# функция возвращает : ( <data> , <сrc> )
# msg_wr - строка данных для передачи инвертору через СОМ-порт
# data - строка данных, полученных от инвертора через СОМ-порт (без CRC и \r)
# crc - 2 символа, соответствующие CRC16 для данных, полученных от инвертора через СОМ-порт

def comm_inverter(msg_wr):
    ser.write(bytes(msg_wr, 'utf-8'))
    time.sleep(0.5)
    data = ser.readline()[:-3]
    crc = ser.readline()[-3:-1]
    return data, crc

# РАСЧЕТ СRC16
# функция возвращает строку вида '\x00\x00', соответствующую 2-м символам СRC16 
# расчет по алгоритму CRC-CCITT (XModem)
# message - строка данных, для которой расчитывается CRC

def crc16(message):
    # https://bytes.com/topic/python/insights/887357-python-check-crc-frame-crc-16-ccitt      
    #CRC-16-CITT poly, the CRC sheme used by ymodem protocol
    #16bit operation register, initialized to zeros
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
            crc_h = reg / 0x100
            crc_l = reg & 0xFF    
    return '\\' + str(hex(crc_h))[1:] + '\\' + str(hex(crc_l))[1:]


    

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

def on_connect(client, userdata, flags, rc):
    #print('Connected...', 'CLIENT:', client, 'USERDATA:', userdata, 'FLAGS:', flags, 'CODE =', rc)
    client.subscribe(topic+'/#')

# ПОЛУЧЕНИЕ КОМАНД ОТ MQTT-БРОКЕРА И ПЕРЕДАЧА ИНВЕРТОРУ
# при получении от MQTT-брокера сообщения из топика, на который оформлена подписка
# полученное сообщение интерпретируется, как команда для передачи инвертору 
# команда передается инвертору через СОМ-порт
# полученное от инвертора подтверждение от инвертора публикуется на MQTT-брокере  
# client - выходная переменная идентификации клиента
# userdata - выходная переменная ...
# msg - полученное сообщение:
#        msg.topic - подписка
#        msg.payload - тело сообщения
#        msg.qos - "качество обслуживания"

def on_message(client, userdata, msg):
    #print('Received message...', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', msg.payload, 'QoS =', msg.qos)
    # value = str(msg.payload)[2:-1]
    value = (msg.payload)[2:-1].decode()
    set_value = '{:0>4.1f}'.format(float(value))
    if msg.topic == topic+'/set/period_s':                              # период опроса инвертора, s
            #print('During :', cmd)
            set_time = int(value)  
       
    if state_read :
        if [msg.topic == topic+'/set/device_source_range' and           # 'PGR'+<value>+<crc16>
            ((value == 'APP' and source_range != 'APP') or
             (value == 'UPS' and source_range != 'UPS'))] :             
            #print('SET_source_range :', value)
            ack = comm_inverter(PGR[value])
            if ack == ('(ACK', '\x39\x20') :
                set_source_range = true
                #print('SETTTING device_source_range OK')
            else :
                set_source_range = false
                #print('SETTTING device_source_range ERR')
            client.publish(topic+'/ack/device_source_range', set_source_range, 0)
                             
        elif [msg.topic == topic+'/set/source_priority' and             # 'POP'+<value>+<crc16>
              ((value == 'UTI' and source_priority != 'UTI') or
               (value == 'SOL' and source_priority != 'SOL') or
               (value == 'SBU' and source_priority != 'SBU'))] :
            #print('Set_source_priority :', value)
            ack = comm_inverter(POP[value])
            if ack == ('(ACK', '\x39\x20') :
                set_source_priority = true
                #print('SETTTING source_priority OK')
            else :
                set_source_priority = false
                #print('SETTTING source_priority ERR')
            client.publish(topic+'/ack/source_priority', set_source_range, 0)
                                
        elif [msg.topic == topic+'/set/charger_priority' and              # 'PCP'+<cmd>+<crc16>
              ((value == 'UTI' and charger_priority != 'UTI') or
               (value == 'SOL' and charger_priority != 'SOL') or
               (value == 'SOL+UTI' and charger_priority != 'SOL+UTI') or
               (value == 'OnlySOL' and charger_priority != 'OnlySOL'))] :
            #print('SET_charger_priority :', value)
            ack = comm_inverter(PCP[value])
            if ack == ('(ACK', '\x39\x20') :
                set_charger_priority = true
                #print('SETTTING charger_priority OK')
            else :
                set_charger_priority = false
                #print('SETTTING charger_priority ERR')
            client.publish(topic+'/ack/charger_priority', set_charger_priority, 0)
                
        elif [msg.topic == topic+'/set/batt_recharge_voltage' and         # 'PBCV'+<set_value>+<crc16>
              batt_recharge_voltage != set_value] :
            #print('SET_batt_recharge_voltage :', value)
            try:
                ack = comm_inverter(PBCV[set_value])
                if ack == ('(ACK', '\x39\x20') :
                    set_batt_recharge_voltage = true
                    #print('SETTTING batt_recharge_voltage OK')
                else :
                    set_batt_recharge_voltage = false
                    #print('SETTTING batt_recharge_voltage ERR')
            except:
                set_batt_recharge_voltage = false
                #print('SET batt_recharge_voltage VAL WRONG')
            client.publish(topic+'/ack/batt_recharge_voltage', set_batt_recharge_voltage, 0)
                    
        elif [msg.topic == topic+'/set/batt_under_voltage' and            # 'PSDV'+<set_value>+<crc16>
            batt_under_voltage != set_value] :
            #print('SET_batt_under_voltage :', value) 
            try:
                ack = comm_inverter(PSDV[set_value])
                if ack == ('(ACK', '\x39\x20') :
                    set_batt_under_voltage = true
                    #print('SETTTING batt_under_voltage OK')
                else :
                    set_batt_under_voltage = false
                    #print('SETTTING batt_under_voltage ERR')
            except:
                set_batt_under_voltage = false
                #print('SET batt_under_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_under_voltage', set_batt_under_voltage, 0)   
                 
        elif [msg.topic == topic+'/set/batt_redischarge_voltage' and      # 'PBDV'+<set_value>+<crc16>
            batt_redischarge_voltage != set_value] :
            #print('SET_batt_redischarge_voltage :', value) 
            try:
                ack = comm_inverter(PBDV[set_value])
                if ack == ('(ACK', '\x39\x20') :
                    set_batt_redischarge_voltage = true
                    #print('SETTTING batt_redischarge_voltage OK')
                else :
                    set_batt_redischarge_voltage = false
                    #print('SETTTING batt_redischarge_voltage ERR')
            except:
                set_batt_redischarge_voltage = false
                #print('SET batt_redischarge_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_redischarge_voltage', set_batt_redischarge_voltage, 0)   
 

# ПОДТВЕРЖДЕНИЕ ПУБЛИКАЦИИ НА MQTT-БРОКЕРЕ

def on_publish(client, userdata, mid):
    # print('Publish OK...', 'CLIENT:', client, 'USERDATA:', userdata, 'MID =', mid)
    return


# КЛИЕНТ MQTT

client = mqtt.Client('EASUN_1234567')
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

time_sta = time.time + set_time

while True

    if (time.time - time_sta) >= set_time :
        time_sta = time.time

# - выходные параметры инвертора

        values = comm_inverter(QPIGS)
        if values[1] == str_crc16(values[0]) :
 
            status_rd = True
            grid_voltage = values[0][1:5]
            grid_frequency = values[0][7:10]
            ac_voltage = values[0][12:16]
            ac_frequency = values[0][18:21]
            ac_va_power = values[0][23:26]
            ac_w_power = values[0][28:31]
            ac_load = values[0][33:35]
            bus_voltage = values[0][37:39]
            batt_voltage = values[0][41:45]
            batt_charging = '{:0>4.1f}'.format(float(values[0][47:49])/10.0)
            batt_capacity = values[0][51:53]
            temp_inverter = values[0][55:58]
            pv_current = values[0][60:63]
            pv_voltage = values[0][65:69]
            pv_power = '{:0>4.0f}'.format(float(pv_current) * float(pv_voltage))
            scc_voltage = values[0][71:75]
            batt_discharge = values[0][77:81]
            device_status = values[0][83:90]
    
            if values[0][86] == '1' : load = 'on' 
            else : load = 'off' 
     
            if values[0][88:90] == '110' : charging = 'Charging on with SCC charge on'
            elif values[0][88:90] == '101' : charging = 'Charging on with AC charge on'
            elif values[0][88:90] == '111' : charging = 'Charging on with SCC and AC charge on',
            else : charging = 'unknow'
    
   
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
    
     
        else:
            #print('QPIGS - COMM ERROR')
            pass

# - режим работы инвертора

        values = comm_inverter(QMOD)
        if values[1] == str_crc16(values[0]) :
 
            if values[0] == '(P' : mode = 'Pover On mode'
            if values[0] == '(S' : mode = 'Standby mode'
            if values[0] == '(L' : mode = 'Line mode'
            if values[0] == '(B' : mode = 'Battery mode'
            if values[0] == '(F' : mode = 'Fault mode'
            if values[0] == '(H' : mode = 'Pover saving mode'    
    
            client.publish(topic+'/info/source_priority', mode , 0)

        else:
            #print('QPIGS - COMM ERROR')
            pass

# - состояние инвертора
   
        values = comm_inverter(QPIRI)
        if values[1] == str_crc16(values[0]) :
   
            if values[0][72] == '0' : source_range = 'APP'
            elif values[0][72] == '1' : source_range = 'UPS' 
            else : source_range = '---'
        
            if values[0][74] == '0' : source_priority = 'UTI'
            elif values[0][74] == '1' : source_priority = 'SOL'
            elif values[0][74] == '2' : source_priority = 'SBU'     
            else : source_priority = '---' 
        
            if values[0][74] == '0' : charger_priority = 'UTI'
            elif values[0][74] == '1' : charger_priority = 'SOL'
            elif values[0][74] == '2' : charger_priority = 'SOL+UTI'
            elif values[0][74] == '3' : charger_priority = 'OnlySOL'       
            else : charger_priority = '---'     
      
            batt_recharge_voltage = values[0][43:46]
            batt_under_voltage = values[0][48:51] 
            batt_redischarge_voltage = values[0][87:90]
             
            client.publish(topic+'/info/source_range', source_range , 0)
            client.publish(topic+'/info/source_priority', source_priority , 0)
            client.publish(topic+'/info/charger_priority', charger_priority , 0)
            client.publish(topic+'/info/batt_recharge_voltage', batt_recharge_voltage , 0)
            client.publish(topic+'/info/batt_under_voltage', batt_under_voltage , 0)
            client.publish(topic+'/info/batt_redischarge_voltage', batt_redischarge_voltage , 0)
      
        else:
            #print('QPIRI - COMM ERROR')
            pass      

# - ошибки и неисправности инвертора
    
        values = comm_inverter(QPIWS)
        if values[1] == str_crc16(values[0]) :
 
            flt = 'Warning'
            if values[0][1] == '1' : client.publish(topic+'/alarm', 'Reserved' , 0)
            if values[0][2] == '1' : flt ='Fault' ; client.publish(topic+'/alarm', 'Inverter fault' , 0)
            if values[0][3] == '1' : client.publish(topic+'/alarm', 'Fault : Bus Over' , 0)
            if values[0][4] == '1' : client.publish(topic+'/alarm', 'Fault : Bus Under' , 0)
            if values[0][5] == '1' : client.publish(topic+'/alarm', 'Fault : Bus Soft Fail' , 0)
            if values[0][6] == '1' : client.publish(topic+'/alarm', 'Line Fail' , 0)
            if values[0][7] == '1' : client.publish(topic+'/alarm', 'OPVShort' , 0)
            if values[0][8] == '1' : client.publish(topic+'/alarm', 'Fault : Inverter voltage too low' , 0)
            if values[0][9] == '1' : client.publish(topic+'/alarm', 'Fault : Inverter voltage too high' , 0)
            if values[0][10] == '1' : client.publish(topic+'/alarm', flt + ' : Over temperature' , 0)
            if values[0][11] == '1' : client.publish(topic+'/alarm', flt + ' : Fan locked' , 0)
            if values[0][12] == '1' : client.publish(topic+'/alarm', flt + ' : Battery voltage high' , 0) 
            if values[0][13] == '1' : client.publish(topic+'/alarm', 'Warning : Battery low alarm' , 0)
            if values[0][14] == '1' : client.publish(topic+'/alarm', 'Reserved' , 0)
            if values[0][15] == '1' : client.publish(topic+'/alarm', 'Warning : Battery under shutdown' , 0)
            if values[0][16] == '1' : client.publish(topic+'/alarm', 'Reserved' , 0)
            if values[0][17] == '1' : client.publish(topic+'/alarm', flt + ' : Over load' , 0)
            if values[0][18] == '1' : client.publish(topic+'/alarm', 'Warning : Eeprom fault' , 0)
            if values[0][19] == '1' : client.publish(topic+'/alarm', 'Fault : Inverter Over Current' , 0)
            if values[0][20] == '1' : client.publish(topic+'/alarm', 'Fault : Inverter Soft Fail' , 0)
            if values[0][21] == '1' : client.publish(topic+'/alarm', 'Fault : Self Test Fail' , 0)
            if values[0][22] == '1' : client.publish(topic+'/alarm', 'Fault : OP DC Voltage Over' , 0) 
            if values[0][23] == '1' : client.publish(topic+'/alarm', 'Fault : Bat Open' , 0)
            if values[0][24] == '1' : client.publish(topic+'/alarm', 'Fault : Current Sensor Fail' , 0)
            if values[0][25] == '1' : client.publish(topic+'/alarm', 'Fault : Battery Short' , 0)
            if values[0][26] == '1' : client.publish(topic+'/alarm', 'Warning : Power limit' , 0)
            if values[0][27] == '1' : client.publish(topic+'/alarm', 'Warning : PV voltage high' , 0)
            if values[0][28] == '1' : client.publish(topic+'/alarm', 'Warning : MPPT overload fault' , 0)
            if values[0][29] == '1' : client.publish(topic+'/alarm', 'Warning : MPPT overload warning' , 0)
            if values[0][30] == '1' : client.publish(topic+'/alarm', 'Warning : Battery too low to charge' , 0)
            if values[0][31] == '1' : client.publish(topic+'/alarm', 'Reserved' , 0)
            if values[0][32] == '1' : client.publish(topic+'/alarm', 'Reserved' , 0)  
 
        else:
            #print('QPIWS - COMM ERROR')
            pass
    
