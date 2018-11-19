import paho.mqtt.client as mqtt
import time
 
broker = "www.mqtt-dashboard.com"
topic = "my_solar"

# КОМАНДЫ УПРАВЛЕНИЯ ИНВЕРТОРОМ (и скриптом)

SET_source_range = " "                         # (APP/UPS)
SET_source_priority = " "                       # (SOL/UTI/SBU)
SET_charger_priority = " "                      # (UTIL/SOL/SOL+UTIL/OnlySOL)	
SET BATT_recharge_voltage = 0.0          # xx.x, V
SET_BATT_under_voltage = 0.0              # xx.x, V
SET_BATT_redischarge_voltage = 0.0      # xx.x, V
SET_Time = 10                                      # периодичность опроса параметров инвертора xx, s 


# ВЫХОДНЫЕ ПАРАМЕТРЫ ИНВЕРТОРА

GRID_voltage = 0.0                             # напряжение сети xxx.x, V
GRID_frequency = 0.0                         # частота сети хх.х, Hz
AC_voltage = 0.0                                # выходное напряжение инвертора ххх.х, V
AC_frequency = 0.0                            # частота на выходе инвертора хх.х, Hz
AC_va_power = 0                                # полная выходная мощность хххх, VA
AC_w_power = 0                                # активная выходная мощность хххх, W
AC_load = 0                                       # нагрузка инвертора ххх, %
BUS_voltage = 0                                 # напряжение шины постоянного тока ххх, V
BATT_voltage = 0.0 
BATT_charging = 0
BATT_capacity = 0                              # емкость батареи xxx, %
TEMP_inverter = 0                               # температура инвертора xxxx, T
PV_current = 0                                    # выходной ток солнечных панелей xxxx, A
PV_voltage                                          # выходное напряжение солнечных панелей xxx.x, V
SCC_voltage                                        # напряжение заряда от солнечных панелей xx.xx, V
BATT_discharge                                   # разрядный ток от аккумулятора ххх, А

# ПАРАМЕТРЫ СОСТОЯНИЯ ИНВЕРТОРА

Source_range = " "               # (APP/UPS)			
Source_priority = " "             # (SOL/UTI/SBU)		
Charger_priority = " "            # (UTIL/SOL/SOL+UTIL/OnlySOL)	
MODE = " "                         # (P,S,L,B,F,H)				
LOAD_on = false					
CHARGING = " "                  # (SCC,AC,SCC+AC)		
GRID_rating_voltage = 0.0     # xxx.x, V
GRID_rating_current = 0.0     # xx.x, A		
		AC_rating_voltage				xxx.x	V
		AC_rating_current				 xx.x	A		
		AC_rating_frequency			 	 xx.x	Hz
		AC_rating_apparent_power		 xxxx	VA
		AC_rating active_power	 	 	 xxxx	W
		BATT_rating_voltage				xx.xx	V
		BATT_recharge voltage			xx.xx	V	
		BATT_under_voltage				xx.xx	V		
		BATT_bulk_voltage				xx.xx	V		
		BATT_float_voltage				xx.xx	V




cmd = 0
s = 258

# СОЕДИНЕНИЕ С БРОКЕРОМ И ПОДПИСКА НА ВЫБРАННЫЕ ТОПИКИ

def on_connect(client, userdata, flags, rc):
    print("Connected...") 
    print("CLIENT: ", client)
    print("USERDATA: ", userdata)
    print("FLAGS: ", flags)
    print("CODE: "+str(rc))
    client.subscribe(topic+'/#')

# ПОЛУЧЕНИЕ И ОБРАБОТКА СООБЩЕНИЙ ОТ БРОКЕРА

def on_message(client, userdata, msg):
    print("Received message ")
    print("CLIENT: ", client)
    print("USERDATA: ", userdata)
    print("TOPIC: ", msg.topic)
    print("MESSAGE: " + str(msg.payload) + " with QoS " + str(msg.qos))

    str_cmd = str(msg.payload)[2:-1]
    if msg.topic == topic+'/cmd/Device_source_range':
        #print("Device_source_range :" + str_cmd)
        if str_cmd == "APP":
            cmd = 0
            
        elif str_cmd == "UPS" :
            cmd = 1
            
        print("Device_source_range CMD:" + str(cmd))
			
    if msg.topic == 'my_solar/cmd/Source_priority':
        #print("Source_priority : " + str_cmd)
        if str_cmd == 'SOL':
            cmd = 0

        elif str_cmd == 'UTI':
            cmd = 1
            
        elif str_cmd == 'SBU':
            cmd = 2
            
        print("Source_priority CMD:" + str(cmd))

# ПОЛУЧЕНИЕ ПОДТВЕРЖДЕНИЯ О ПУБЛИКАЦИИ ТОПИКА

def on_publish(client, userdata, mid):
    # print("Publish OK...")
    # print("CLIENT: ", client)
    # print("USERDATA: ", userdata)
    # print("MID: " + str(mid))

# КЛИЕНТ MQTT

client = mqtt.Client("EASUN_1234567")
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

    client.publish("my_solar/info/AC_voltage", str(s) , 0)
    client.publish("my_solar/cmd/Source_priority", "SOL" , 0)


    
