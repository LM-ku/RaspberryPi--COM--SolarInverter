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

SET_source_range = ''                           # (APP/UPS)
SET_source_priority = ''                        # (SOL/UTI/SBU)
SET_charger_priority = ''                       # (UTI/SOL/SOL+UTI/OnlySOL)	
SET_BATT_recharge_voltage = 0.0                 # xx.x, V
SET_BATT_under_voltage = 0.0                    # xx.x, V
SET_BATT_redischarge_voltage = 0.0              # xx.x, V
SET_Time = 10                                   # периодичность опроса инвертора xx, s 


# ВЫХОДНЫЕ ПАРАМЕТРЫ ИНВЕРТОРА

GRID_voltage = 0.0                              # напряжение сети xxx.x, V
GRID_frequency = 0.0                            # частота сети хх.х, Hz
AC_voltage = 0.0                                # выходное напряжение инвертора ххх.х, V
AC_frequency = 0.0                              # частота на выходе инвертора хх.х, Hz
AC_va_power = 0                                 # полная выходная мощность хххх, VA
AC_w_power = 0                                  # активная выходная мощность хххх, W
AC_load = 0                                     # нагрузка инвертора ххх, %
BUS_voltage = 0                                 # напряжение шины постоянного тока ххх, V
BATT_voltage = 0.0 
BATT_charging = 0
BATT_capacity = 0                               # емкость батареи xxx, %
TEMP_inverter = 0                               # температура инвертора xxxx, T
PV_current = 0                                  # выходной ток солнечных панелей xxxx, A
PV_voltage                                      # выходное напряжение солнечных панелей xxx.x, V
PV_power                                        # выходная солнечная мощность xxxx, W
SCC_voltage                                     # напряжение заряда от солнечных панелей xx.xx, V
BATT_discharge                                  # разрядный ток от аккумулятора ххх, А

# ИНФОРМАЦИЯ О СОСТОЯНИИ ИНВЕРТОРА

Source_range = 0                                # (APP/UPS)			
Source_priority = 0                             # (SOL/UTI/SBU)		
Charger_priority = 0                            # (UTIL/SOL/SOL+UTIL/OnlySOL)	
MODE = ' '                                      # (P,S,L,B,F,H)				
LOAD_on = false					
CHARGING = ' '                                  # (SCC,AC,SCC+AC)		
GRID_rating_voltage = 0.0                       # xxx.x, V
GRID_rating_current = 0.0                       # xx.x, A		
#		AC_rating_current				 xx.x	A		
#		AC_rating_frequency			 	 xx.x	Hz
#		AC_rating active_power	 	 	 xxxx	W
#		BATT_rating_voltage				xx.xx	V
#		BATT_recharge voltage			xx.xx	V	
#		BATT_under_voltage				xx.xx	V		
#		BATT_bulk_voltage				xx.xx	V		
#		BATT_float_voltage				xx.xx	V


# ФЛАГИ КОМАНД ИНВЕРТОРА 

PGR = false
POP = false
PSP = false
PBCV = false
PSDV = false
PBDV = false

cmd = ''
s = 258
state_read = false

# РАСЧЕТ СRC16

def str_CRC16(message):
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

# ПОЛУЧЕНИЕ КОМАНД ОТ БРОКЕРА И ПЕРЕДАЧА ИНВЕРТОРУ

def on_message(client, userdata, msg):
    #print('Received message...', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', msg.payload, 'QoS =', msg.qos)
    cmd = str(msg.payload)[2:-1]
    if state_read :
            
            if msg.topic == topic+'/set/period_s':        
                #print('During :', cmd)
                SET_Time = int(cmd)    

            if msg.topic == topic+'/set/device_source_range':           # 'PGR'+SET_source_range+<crc16>
                PGR = true
                if cmd == 'APP': SET_source_range = '00'
                elif cmd == 'UPS': SET_source_range = '01'
                #print('Source_range :', cmd)
        
            if msg.topic == topic+'/set/source_priority':               # 'POP'+SET_source_range+<crc16>   
                POP = true
                if cmd == 'UTI': SET_source_priority = '00'
                elif cmd == 'SOL': SET_source_priority = '01'
                elif cmd == 'SBU': SET_source_priority = '02'
                #print('Source_priority :', cmd)
        
            if msg.topic == topic+'/set/charger_priority':              # 'PSP'+SET_charger_priority+<crc16>   
                PSP = true
                if cmd == 'UTI': SET_charger_priority = '00'
                elif cmd == 'SOL': SET_charger_priority = '01'
                elif cmd == 'SOL+UTI': SET_charger_priority = '02'
                elif cmd == 'OnlySOL': SET_charger_priority = '03'
                #print('Charger_priority :', cmd)

            if msg.topic == topic+'/set/batt_recharge_voltage':         # 'PBCV'+SET BATT_recharge_voltage+<crc16>
                PBCV = true
                lower_limit = 44.0
                uper_limit = 51.0
                SET_BATT_recharge_voltage = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('BATT_recharge_voltage :', SET_BATT_recharge_voltage)

            if msg.topic == topic+'/set/batt_under_voltage':            # 'PSDV'+SET BATT_under_voltage+<crc16>
                PSDV = true
                lower_limit = 40.0
                uper_limit = 48.0
                SET_BATT_under_voltage = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('BATT_under_voltage :', SET_BATT_under_voltage)

            if msg.topic == topic+'/set/batt_redischarge_voltage':            # 'PBDV'+SET BATT_redischarge_voltage+<crc16>
                PSDV = true
                lower_limit = 48.0
                uper_limit = 58.0
                SET_BATT_redischarge_voltage = to_value(cmd, lower_limit, uper_limit, '{:0>4.1f}')
                #print('BATT_redischarge_voltage :', SET_BATT_redischarge_voltage)



        
        

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
