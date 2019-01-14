import paho.mqtt.client as mqtt
#import serial
import time
 
#broker = 'www.mqtt-dashboard.com'
broker = 'localhost'
topic = 'my_solar'



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

source_range = 'APP'
source_priority = 'UTI'
charger_priority = 'UTI'
batt_recharge_voltage = 44.0
batt_under_voltage = 42.0
batt_redischarge_voltage = 52.0

ct_qmod = 0
ct_stat = 0
ct_qpiws = 0
ct_load = 0
ct_charge = 0

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
            source_range = value
            set_source_range = True
            #print('SETTTING device_source_range OK')
            client.publish(topic+'/ack/device_source_range', set_source_range, 0)
                             
        elif [msg.topic == topic+'/set/source_priority' and             # 'POP'+<value>+<crc16>
              ((value == 'UTI' and source_priority != 'UTI') or
               (value == 'SOL' and source_priority != 'SOL') or
               (value == 'SBU' and source_priority != 'SBU'))] :
            #print('Set_source_priority :', value)
            source_priority = value
            set_source_priority = True
            #print('SETTTING source_priority OK')
            client.publish(topic+'/ack/source_priority', set_source_range, 0)
                                
        elif [msg.topic == topic+'/set/charger_priority' and              # 'PCP'+<cmd>+<crc16>
              ((value == 'UTI' and charger_priority != 'UTI') or
               (value == 'SOL' and charger_priority != 'SOL') or
               (value == 'SOL+UTI' and charger_priority != 'SOL+UTI') or
               (value == 'OnlySOL' and charger_priority != 'OnlySOL'))] :
            #print('SET_charger_priority :', value)
            charger_priority = value
            #print('SETTTING charger_priority OK')
            client.publish(topic+'/ack/charger_priority', set_charger_priority, 0)
                
        elif [msg.topic == topic+'/set/batt_recharge_voltage' and         # 'PBCV'+<set_value>+<crc16>
              batt_recharge_voltage != set_value] :
            #print('SET_batt_recharge_voltage :', value)
            try:
                batt_recharge_voltage = set_value
                set_batt_recharge_voltage = True
                #print('SETTTING batt_recharge_voltage OK')
            except:
                set_batt_recharge_voltage = False
                #print('SET batt_recharge_voltage VAL WRONG')
            client.publish(topic+'/ack/batt_recharge_voltage', set_batt_recharge_voltage, 0)
                    
        elif [msg.topic == topic+'/set/batt_under_voltage' and            # 'PSDV'+<set_value>+<crc16>
            batt_under_voltage != set_value] :
            #print('SET_batt_under_voltage :', value) 
            try:
                batt_under_voltage = set_value
                set_batt_under_voltage = True
                #print('SETTTING batt_under_voltage OK')
            except:
                set_batt_under_voltage = False
                #print('SET batt_under_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_under_voltage', set_batt_under_voltage, 0)   
                 
        elif [msg.topic == topic+'/set/batt_redischarge_voltage' and      # 'PBDV'+<set_value>+<crc16>
            batt_redischarge_voltage != set_value] :
            #print('SET_batt_redischarge_voltage :', value) 
            try:
                batt_redischarge_voltage = set_value
                set_batt_redischarge_voltage = True
                #print('SETTTING batt_redischarge_voltage OK')
            except:
                set_batt_redischarge_voltage = False
                #print('SET batt_redischarge_voltage VAL WRONG') 
            client.publish(topic+'/ack/batt_redischarge_voltage', set_batt_redischarge_voltage, 0)   
 

# ПОДТВЕРЖДЕНИЕ ПУБЛИКАЦИИ НА MQTT-БРОКЕРЕ

def on_publish(client, userdata, mid):
    # print('Publish OK...', 'CLIENT:', client, 'USERDATA:', userdata, 'MID =', mid)
    return

# СЕРИЙНЫЙ НОМЕР ИНВЕРТОРА

device_serial = 'xxxxxxxxxxxxxx'


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
       
        if (ct_qmod >= 4) :
            if qmod == '(P': qmod = '(S' ; ct_qmod = 0
            elif qmod == '(S': qmod = '(L' ; ct_qmod = 0
            elif qmod == '(L': qmod = '(B' ; ct_qmod = 0
            elif qmod == '(B': qmod = '(F' ; ct_qmod = 0
            elif qmod == '(F': qmod = '(H' ; ct_qmod = 0
            else: qmod = '(P' ; ct_qmod = 0 ; answer = qmod
        ct_qmod =+ 1
        
        device_mode = mode[qmod]
        client.publish(topic+'/mode/mode', device_mode , 0)
        client.publish(topic+'/mode/QMOD_comm', comm_state , 0)
        
# - параметры инвертора

        grid_voltage = 220.0 + random.uniform(-20, 20)
        grid_frequency = 50.0 + random.uniform(-2, 2)
        
        if mode == 'Batt mode' : ac_voltage = 230.0 + random.uniform(-2, 2)
        if mode == 'Line mode' : ac_voltage = grid_voltage
        else : ac_voltage = 0.0
        
        if mode == 'Batt mode' : ac_frequency = 50.0 + random.uniform(-1, 1)
        if mode == 'Line mode' : ac_frequency = grid_frequency
        else : ac_frequency = 0.0

        ac_load = 75 + random.randint(-25, 25)    
        ac_va_power = output_load_percent * 40
        ac_w_power = int(ac_output_apparent_power * 0.85)
        bus_voltage = 420 + random.randint(-5, 5)
        
        
        batt_voltage = 48.00 + random.uniform(-6, 6)
        batt_charging = 100 + random.randint(-50, 50)
        batt_capacity = int(100 * battery_voltage / 56.0)
        temp_inverter = 20 + random.randint(-5, 15)
        pv_current = 15 + random.randint(-10, 5)
        pv_voltage = 72.0 + random.uniform(-20, 10)
        pv_power = pv_current * pv_voltage
        scc_voltage = battery_voltage + 2
        batt_discharge = 100 + random.randint(-20, 20)

        
         
        device_status = data[83:91]
    
        if device_status[3] == '1' : load = 'on' 
        if device_status[3] == '0' : load = 'off' 
     
        if device_status[5:] == '000' : charging = 'NOT charging'
        if device_status[5:] == '110' : charging = 'Charging with SCC'
        if device_status[5:] == '101' : charging = 'Charging with AC grid'
        if device_status[5:] == '111' : charging = 'Charging with SCC + AC grid'


   
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
