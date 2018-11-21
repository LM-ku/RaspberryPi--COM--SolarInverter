import paho.mqtt.client as mqtt
import time
 
# broker = 'www.mqtt-dashboard.com'
broker = 'localhost'
topic = 'my_solar'

# КОМАНДЫ УПРАВЛЕНИЯ ИНВЕРТОРОМ (и скриптом)

SET_source_range = ' '                              # (APP/UPS)
SET_source_priority = ' '                           # (SOL/UTI/SBU)
SET_charger_priority = ' '                          # (UTI/SOL/SOL+UTI/OnlySOL)	
SET_BATT_recharge_voltage = '00.0'                  # xx.x, V
SET_BATT_under_voltage = '00.0'                     # xx.x, V
SET_BATT_redischarge_voltage = '00.0'               # xx.x, V
SET_Time = 10                                       # периодичность опроса параметров инвертора xx, s 


# ВЫХОДНЫЕ ПАРАМЕТРЫ ИНВЕРТОРА

GRID_voltage = 0.0                                  # напряжение сети xxx.x, V
GRID_frequency = 0.0                                # частота сети хх.х, Hz
AC_voltage = 0.0                                    # выходное напряжение инвертора ххх.х, V
AC_frequency = 0.0                                  # частота на выходе инвертора хх.х, Hz
AC_va_power = 0                                     # полная выходная мощность хххх, VA
AC_w_power = 0                                      # активная выходная мощность хххх, W
AC_load = 0                                         # нагрузка инвертора ххх, %
BUS_voltage = 0                                     # напряжение шины постоянного тока ххх, V
BATT_voltage = 0.0 
BATT_charging = 0
BATT_capacity = 0                                   # емкость батареи xxx, %
TEMP_inverter = 0                                   # температура инвертора xxxx, T
PV_current = 0                                      # выходной ток солнечных панелей xxxx, A
PV_voltage = 0.0                                    # выходное напряжение солнечных панелей xxx.x, V
SCC_voltage = 0.0                                   # напряжение заряда от солнечных панелей xx.xx, V
BATT_discharge = 0                                  # разрядный ток от аккумулятора ххх, А
PV_power = 0                                        # солнечная мощность xxx, W

# ПАРАМЕТРЫ СОСТОЯНИЯ ИНВЕРТОРА

Source_range = 'n/a'                                # (APP/UPS)			
Source_priority = 'n/a'                             # (SOL/UTI/SBU)		
Charger_priority = 'n/a'                            # (UTI/SOL/SOL+UTI/OnlySOL)	
MODE = 'n/a'                                        # (P,S,L,B,F,H)				
LOAD = 'n/a'					    # (ON/OFF)
CHARGING = 'n/a'                                    # (SCC,AC,SCC+AC)		
GRID_rating_voltage = 0.0                           # xxx.x, V
GRID_rating_current = 0.0                           # xx.x, A		
#		AC_rating_voltage				xxx.x	V
#		AC_rating_current				 xx.x	A		
#		AC_rating_frequency			 	 xx.x	Hz
#		AC_rating_apparent_power		 xxxx	VA
#		AC_rating active_power	 	 	 xxxx	W
#		BATT_rating_voltage				xx.xx	V
#		BATT_recharge voltage			xx.xx	V	
#		BATT_under_voltage				xx.xx	V		
#		BATT_bulk_voltage				xx.xx	V		
#		BATT_float_voltage				xx.xx	V




cmd = ''
s = 258


def value_to_set(x, lower_limit, upper_limit, form):
    out = float(x)
    if out > h : out = upper_limit
    if out < l : out = lower_limit
    return (form.format(out))     


# СОЕДИНЕНИЕ С БРОКЕРОМ И ПОДПИСКА НА ВЫБРАННЫЕ ТОПИКИ

def on_connect(client, userdata, flags, rc):
    # print('Connected...','CLIENT:', client, 'USERDATA:', userdata, 'FLAGS:', flags, 'CODE:', rc)
    client.subscribe(topic+'/#')

# ПОЛУЧЕНИЕ И ОБРАБОТКА СООБЩЕНИЙ ОТ БРОКЕРА

def on_message(client, userdata, msg):
    # print('Received message ', 'CLIENT:', client, 'USERDATA:', userdata, 'TOPIC:', msg.topic, 'MESSAGE:', str(msg.payload), 'with QoS =', str(msg.qos))
    cmd = str(msg.payload)[2:-1]
    if msg.topic == topic+'/set/device_source_range':
        SET_source_range = cmd
        #print('SET Device_source_range : ' + SET_source_range)
        
    if msg.topic == 'my_solar/set/source_priority':
        SET_source_priority = cmd
        #print('SET Source_priority : ' + SET_source_priority)
        
    if msg.topic == 'my_solar/set/charger_priority':
        SET_charger_priority = cmd
        #print('SET Charger_priority : ' + SET_charger_priority)
 
    if msg.topic == 'my_solar/set/batt_recharge_voltage':
        SET_BATT_recharge_voltage = value_to_set(cmd, 44.0, 51.0, '{:0>4.1f}')
        #print('SET BATT_recharge_voltage : ' + SET_BATT_recharge_voltage)
       
    if msg.topic == 'my_solar/set/batt_under_voltage':
        SET_BATT_under_voltage = value_to_set(cmd, 40.0, 48.0, '{:0>4.1f}')
        #print('SET BATT_under_voltage : ' + SET_BATT_under_voltage)
        
    if msg.topic == 'my_solar/set/batt_redischarge_voltage':
        SET_BATT_redischarge_voltage = value_to_set(cmd, 48.0, 58.0, '{:0>4.1f}')
        #print('SET BATT_redischarge_voltage : ' + SET_BATT_redischarge_voltage)
        

        

# ПОДТВЕРЖДЕНИЯ О ПУБЛИКАЦИИ ТОПИКА

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

# ЦИКЛ ОПРОСА ИНВЕРТОРА И ПЕРЕДАЧИ ДАННЫХ

while True:
    time.sleep(SET_Time)

# publish(topic, payload, wait_for_publish)
# wait_for_publish == 0 - публикация вне зависимости от наличия связи с брокером, данные могут быть потеряны
# wait_for_publish == 1 - публикация состоится только при наличии связи с брокером, все данные будут опубликованы после соединения с брокером

    client.publish(topic + '/info/AC_voltage', str(s) , 0)
    client.publish(topic + '/cmd/Source_priority', 'SOL' , 0)


    
