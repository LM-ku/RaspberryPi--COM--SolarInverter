
import serial
import time
import random 



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
            crc_l = reg & 0xFF
            crc_h = reg >> 8            
    return chr(crc_h) + chr(crc_l)

qmod = '(P'
ct_qmod = 0
ct_qpiws = 0

# ОПРЕДЕЛЕНИЕ СОМ-ПОРТА ДЛЯ ОБМЕНА С ИНВЕРТОРОМ

try :
    ser = serial.Serial(              
        port='COM7',
        baudrate = 2400,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=0.1
        )
    print('COM-порт ОТКРЫТ !') 

except Exception:
    print('Проверьте СОМ-порт !!!')

else :
    
# ПРОСЛУШИВАНИЕ СОМ-ПОРТА И ЭМУЛЯЦИЯ ОТВЕТА ИНВЕРТОРА

    try :

        while True :
        
            while ser.inWaiting():
            
                rd = ser.readline()
                print(rd)
                rd_serial = rd.decode()
                print(rd_serial)
                data = rd_serial[:-3]
                crc = rd_serial[-3:-1]

                if crc == crc16(data) :
                    print('read serial :', rd_serial)
                    print('       data =', data, '  crc =', crc)

# - номер инвертора

                    if data == 'QID': answer = '(EASUN_12345678'
 

# - режим работы инвертора

                    if (data == 'QMOD') :
                        if (ct_qmod >= 4) :
                            if qmod == '(P': qmod = '(S' ; ct_qmod = 0
                            elif qmod == '(S': qmod = '(L' ; ct_qmod = 0
                            elif qmod == '(L': qmod = '(B' ; ct_qmod = 0
                            elif qmod == '(B': qmod = '(F' ; ct_qmod = 0
                            elif qmod == '(F': qmod = '(H' ; ct_qmod = 0
                            else: qmod = '(P' ; ct_qmod = 0 ; answer = qmod
                        ct_qmod =+ 1
                
# - параметры инвертора

                    if data == 'QPIGS':
                        grid_voltage = 220.0 + random.uniform(-20, 20)
                        grid_frequence = 50.0 + random.uniform(-2, 2)

                        output_load_percent = 75 + random.randint(-25, 25)
                    
                        if qmod == '(B' :
                            ac_output_voltage = 230.0 + random.uniform(-2, 2)
                            ac_output_frequnce = 50.0 + random.uniform(-1, 1)
                        if qmod == '(L' :
                            ac_output_voltage = grid_voltage
                            ac_output_frequnce = grid_frequency
                        else :
                            ac_voltage = 0.0
                            ac_frequency = 0.0
                            output_load_percent = 0

                        ac_output_apparent_power = output_load_percent * 40
                        ac_output_active_power = int(ac_output_apparent_power * 0.85)
                        bus_voltage = 420 + random.randint(-5, 5)
                        battery_voltage = 48.00 + random.uniform(-6, 6)
                        battery_charging_current = 100 + random.randint(-50, 50)
                        battery_capacity = int(100 * battery_voltage / 56.0)
                        inverter_temperature = 20 + random.randint(-5, 15)
                        pv_input_current = 15 + random.randint(-10, 5)
                        pv_input_voltage = 72.0 + random.uniform(-20, 10)
                        scc_voltage = battery_voltage + 2
                        discharge_current = 100 + random.randint(-20, 20)
                        device_status = ' 00010111'
            
                        answer = '('+'{:0>4.1f}'.format(grid_voltage)\
                                 +' {:0>3.1f}'.format(grid_frequence)\
                                 +' {:0>4.1f}'.format(ac_output_voltage)\
                                 +' {:0>3.1f}'.format(ac_output_frequnce)\
                                 +' {:0>4d}'.format(ac_output_apparent_power)\
                                 +' {:0>4d}'.format(ac_output_active_power)\
                                 +' {:0>3d}'.format(output_load_percent)\
                                 +' {:0>3d}'.format(bus_voltage)\
                                 +' {:0>5.2f}'.format(battery_voltage)\
                                 +' {:0>3d}'.format(battery_charging_current)\
                                 +' {:0>3d}'.format(battery_capacity)\
                                 +' {:0>4d}'.format(inverter_temperature)\
                                 +' {:0>4d}'.format(pv_input_current)\
                                 +' {:0>5.1f}'.format(pv_input_voltage)\
                                 +' {:0>5.2f}'.format(scc_voltage)\
                                 +' {:0>5d}'.format(discharge_current)\
                                 + device_status\
                                 +' 00 00 00000 100'
            
# - состояние инвертора

                    if data == 'QPIRI':
                        answer = '(230.0 21.7 230.0 50.0 21.7 5000 4000 48.0 46.0 42.0 56.4 54.0 0 10 010 1 0 0 6 01 0 0 54.0 0 1'

# - ошибки и неисправности инвертора

                    if (data == 'QPIWS') :
                        ct_qpiws = (ct_qpiws << 1) + 1
                        if ct_qpiws > 0b00111111111111111111111111111111 : ct_qpiws = 0
            
                        answer ='({:0>32b}'.format(ct_qpiws)
            
                    msg_wr = answer + crc16(answer)+'\x0D'
                    print('write serial :', msg_wr)
                    print('        data =', answer, '  crc =', msg_wr[-3:-1])
                    ser.write(bytes(msg_wr, 'utf-8'))

                else : print('read serial...')


    except Exception:
        print('COM-PORT ERROR')
#        ser.close()
#        print('Port is closed')




