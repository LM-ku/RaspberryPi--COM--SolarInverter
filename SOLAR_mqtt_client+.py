import paho.mqtt.client as mqtt
import serial
import time
 
#broker = 'www.mqtt-dashboard.com'
broker = 'localhost'
topic = 'my_solar'

# ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ

ser = serial.Serial(              
    port='/dev/ttyAMA0',
    baudrate = 2400,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    bytesize=serial.EIGHTBITS,
    timeout=1
)


# КОМАНДЫ УПРАВЛЕНИЯ ИНВЕРТОРОМ (и скриптом)

set_source_range = ''                           # (APP/UPS)
set_source_priority = ''                        # (SOL/UTI/SBU)
set_charger_priority = ''                       # (UTI/SOL/SOL+UTI/OnlySOL)	
set_batt_recharge_voltage = 0.0                 # xx.x, V
set_batt_under_voltage = 0.0                    # xx.x, V
set_batt_redischarge_voltage = 0.0              # xx.x, V
set_time = 10                                   # периодичность опроса инвертора xx, s 


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

source_range = 0                                # (APP/UPS)			
source_priority = 0                             # (SOL/UTI/SBU)		
charger_priority = 0                            # (UTIL/SOL/SOL+UTIL/OnlySOL)	
mode = ' '                                      # (P,S,L,B,F,H)				
load_on = false					
charging = ' '                                  # (SCC,AC,SCC+AC)		
grid_rating_voltage = 0.0                       # xxx.x, V
grid_rating_current = 0.0                       # xx.x, A		
#		AC_rating_current				 xx.x	A		
#		AC_rating_frequency			 	 xx.x	Hz
#		AC_rating active_power	 	 	 xxxx	W
#		BATT_rating_voltage				xx.xx	V
#		BATT_recharge voltage			xx.xx	V	
#		BATT_under_voltage				xx.xx	V		
#		BATT_bulk_voltage				xx.xx	V		
#		BATT_float_voltage				xx.xx	V


# ФЛАГИ КОМАНД ИНВЕРТОРА 

pgr = false
pop = false
psp = false
pbcv = false
psdv = false
pbdv = false

# КОМАНДЫ & СRC16

QPGS0 = '\x51\x50\x47\x53\x30\x3F\xDa\x0D'               # Parallel Information inquiry（For 4K/5K）
QPIGS = '\x51\x50\x49\x47\x53\xB7\xA9\x0D'               # Device general status parameters inquiry
QMCHGCR = '\x51\x4D\x43\x48\x47\x43\x52\xD8\x55\x0D'     # Enquiry selectable value about max charging current
QMUCHGCR = '\x51\x4D\x55\x43\x48\x47\x43\x52\x26\x34\0D' # Enquiry selectable value about max utility charging current
QPIWS = '\x51\x50\x49\x57\x53\xB4\xDA\x0D'               # Device Warning Status inquiry
POP00 = '\x50\x4F\x50\x30\x30\xC2\x48\x0D'               # Setting device output source priority to UTI
POP01 = '\x50\x4F\x50\x30\x31\xD2\x69\x0D'               # Setting device output source priority to SOL
POP02 = '\x50\x4F\x50\x30\x32\xE2\x0B\x0D'               # Setting device output source priority to SBU
PCP00 = '\x50\x43\x50\x30\x30\x8D\x7A\x0D'               # Setting device charger priority to UTI first
PCP01 = '\x50\x43\x50\x30\x31\x9D\x5B\x0D'               # Setting device charger priority to SOL first
PCP02 = '\x50\x43\x50\x30\x32\xAD\x38\x0D'               # Setting device charger priority to SOL+UTI
PCP03 = '\x50\x43\x50\x30\x33\xBD\x19\x0D'               # Setting device charger priority to OnlySOL
PGR00 = '\x50\x47\x52\x30\x30\x29\xEB\x0D'               # Setting device grid working range to APP
PGR01 = '\x50\x47\x52\x30\x31\x39\xCA\x0D'               # Setting device grid working range to UPS
QMOD = '\x51\x4D\x4F\x44\x49\xC1\x0D'                    # Device Mode inquiry
QDI = '\x51\x44\x49\x71\x1B\x0D'                         # The default setting value information
QVFW = '\x51\x56\x46\x57\x62\x99\x0D'                    # Main CPU Firmware version inquiry
QVFW2 = '\x51\x56\x46\x57\x32\xC3\xF5\x0D'               # Another CPU Firmware version inquiry
QPIRI = '\x51\x50\x49\x52\x49\xF8\x54\x0D'               # Device Rating Information inquiry
QFLAG = '\x51\x46\x4C\x41\x47\x98\x74\x0D'               # Device flag status inquiry 





cmd = ''
s = 258
state_read = false

# РАСЧЕТ СRC16

def str_crc16(message):
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

# ФОРМАТИРОВАНИЕ УСТАВКИ ДЛЯ ПЕРЕДАЧИ В ИНВЕРТОР

def to_value(cmd, lower_limit, uper_limit, pattern):
    if float(cmd) < lower_limit : return(lower_limit)
    if float(cmd) > uper_limit : return(uper_limit)
    return(pattern.format(float(cmd)))


# СОЕДИНЕНИЕ С БРОКЕРОМ И ПОДПИСКА НА ТОПИКИ

def on_connect(client, userdata, flags, rc):
    #print('Connected...', 'CLIENT:', client, 'USERDATA:', userdata, 'FLAGS:', flags, 'CODE =', rc)
    client.subscribe(topic+'/#')

# ПОЛУЧЕНИЕ КОМАНД ОТ БРОКЕРА И ФОРМИРОВАНИЕ СООБЩЕНИЙ ДЛЯ ПЕРЕДАЧИ ИНВЕРТОРУ

def on_message(client, userdata, msg):
    #print('Received message...', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', msg.payload, 'QoS =', msg.qos)
    cmd = str(msg.payload)[2:-1]
    if msg.topic == topic+'/set/period_s':        
            #print('During :', cmd)
            SET_Time = int(cmd)  
       
    if state_read :
            if msg.topic == topic+'/set/device_source_range':             # 'PGR'+set_source_range+<crc16>
                #print('SET_source_range :', cmd)
                if cmd == 'APP' and source_range != '00'   : set_source_range = PGR00 ; pgr = true
                elif cmd == 'UPS' and source_range != '01' : set_source_range = PGR01 ; pgr = true
                else pgr = false
             
            elif msg.topic == topic+'/set/source_priority':              # 'POP'+set_source_range+<crc16>   
                #print('Set_source_priority :', cmd)
                if cmd == 'UTI' and source_priority != '00'   : set_source_priority = POP00 ; pop = true
                elif cmd == 'SOL' and source_priority != '01' : set_source_priority = POP01 ; pop = true
                elif cmd == 'SBU' and source_priority != '02' : set_source_priority = POP02 ; pop = true
                else : pop = false
                
            elif msg.topic == topic+'/set/charger_priority':              # 'PSP'+set_charger_priority+<crc16>  
                #print('SET_charger_priority :', cmd)            
                if cmd == 'UTI' and charger_priority != '00'       : set_charger_priority = PCP00 ; pcp = true
                elif cmd == 'SOL' and charger_priority != '01'     : set_charger_priority = PCP01 ; pcp = true
                elif cmd == 'SOL+UTI' and charger_priority != '02' : set_charger_priority = PCP02 ; pcp = true
                elif cmd == 'OnlySOL' and charger_priority != '03' : set_charger_priority = PCP03 ; pcp = true
                else : pop = false
                
            elif msg.topic == topic+'/set/batt_recharge_voltage':         # 'PBCV'+set_batt_recharge_voltage+<crc16>
                lower_limit = 44.0 ; uper_limit = 51.0
                set_value = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('SET_batt_recharge_voltage :', set_value)
                if batt_recharge_voltage != set_value :
                    message = 'PBCV' + set_value
                    set_batt_recharge_voltage = message + str_crc16(message) + '\x0D'
                    pbcv = true
                else : pbcv = false

            elif msg.topic == topic+'/set/batt_under_voltage':            # 'PSDV'+set_batt_under_voltage+<crc16>
                lower_limit = 40.0 ; uper_limit = 48.0
                set_value = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('SET_batt_under_voltage :', set_value)
                if batt_under_voltage != set_value :
                   message = 'PCDV' + set_value
                   set_batt_under_voltage = message + str_crc16(message) + '\x0D'
                   psdv = true
                else : psdv = false   
                 
            elif msg.topic == topic+'/set/batt_redischarge_voltage':            # 'PBDV'+set_batt_redischarge_voltage+<crc16>
                PSDV = true
                lower_limit = 48.0
                uper_limit = 58.0
                set_batt_redischarge_voltage = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('SET_batt_redischarge_voltage :', SET_BATT_redischarge_voltage)

# ОБМЕН СООБЩЕНИЯМИ С ИНВЕРТОРОМ
def msg_from_inverter(msg_wr):
    ser.write(bytes(msg_wr, 'utf-8'))
    time.sleep(0.5)
    return = str(ser.readline(),'utf-8')  

# ПОДТВЕРЖДЕНИЕ ПУБЛИКАЦИИ ТОПИКА

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






# ЦИКЛ ОПРОСА ИНВЕРТОРА И ПЕРЕДАЧИ ДАННЫХ (ПУБЛИКАЦИИ)

while True:
    time.sleep(SET_Time)

# ОПРОС ИНВЕРТОРА ЧЕРЕЗ RS232









# ПУБЛИКАЦИЯ ВЫХОДНЫХ ПАРАМЕТРОВ ИНВЕРТОРА
# publish(topic, payload, wait_for_publish)
# wait_for_publish == 0 - публикация вне зависимости от наличия связи с брокером, данные могут быть потеряны
# wait_for_publish == 1 - публикация состоится только при наличии связи с брокером, все данные будут опубликованы после соединения с брокером

    client.publish(topic+'/values/grid_voltage', GRID_voltage, 0)
    client.publish(topic+'/values/grid_frequency', GRID_frequency, 0)
    client.publish(topic+'/values/ac_voltage', AC_voltage, 0)
    client.publish(topic+'/values/ac_frequency', AC_frequency, 0)
    client.publish(topic+'/values/ac_va_power', AC_va_power, 0)
    client.publish(topic+'/values/ac_w_power', AC_w_power, 0)
    client.publish(topic+'/values/ac_load', AC_load, 0)
    client.publish(topic+'/values/bus_voltage', BUS_voltage, 0)
    client.publish(topic+'/values/batt_voltage', BATT_voltage, 0)
    client.publish(topic+'/values/batt_charging', BATT_charging, 0)
    client.publish(topic+'/values/batt_capacity', BATT_capacity, 0)
    client.publish(topic+'/values/temp_inverter', TEMP_inverter, 0)
    client.publish(topic+'/values/pv_current', PV_current, 0)
    client.publish(topic+'/values/pv_voltage', PV_voltage, 0)
    client.publish(topic+'/values/pv_power', PV_power, 0)
    client.publish(topic+'/values/scc_voltage', SCC_voltage, 0)
    client.publish(topic+'/values/batt_discharge', BATT_discharge, 0)


# ПУБЛИКАЦИЯ ИНФОРМАЦИИ О СОСТОЯНИИ ИНВЕРТОРА
    
    client.publish(topic+'/info/source_priority', 'SOL' , 0)


# ПУБЛИКАЦИЯ ИНФОРМАЦИИ ОБ ОШИБКАХ ИНВЕРТОРА
    
    client.publish(topic+'/warning/reserved', warning[0] , 0)
    

# ПЕРЕДАЧА КОМАНД ИНВЕРТОРУ

    if PGR and SET_source_range != Source_range :
        message = 'PGR'+SET_source_range
        cmd_write =  message + str_CRC16(message) + '\n'          
