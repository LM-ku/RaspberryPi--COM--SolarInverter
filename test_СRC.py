# import serial


# ПОСЛЕДОВАТЕЛЬНЫЙ ПОРТ

#ser = serial.Serial(              
#    baudrate = 2400,
#    parity=serial.PARITY_NONE,
#    stopbits=serial.STOPBITS_ONE,
#    bytesize=serial.EIGHTBITS,
#    timeout=1
#)


def str_CRC16(message):
    #CRC-16-CITT poly, the CRC sheme used by ymodem protocol
    #poly = 0x11021
    #16bit operation register, initialized to zeros
    #reg = 0xFFFF
    
    poly = 0x1021
    reg = 0
    #pad the end of the message with the size of the poly
    message = str(message)
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
            #crc_h = reg / 0x100
            crc_h = reg >> 8
            crc_l = reg & 0xFF    
    #return '\\' + str(hex(crc_h))[1:] + '\\' + str(hex(crc_l))[1:] + "'"
    return hex(crc_h) , hex(crc_l)
    #return ['{:x}'.format(crc_h) ,'{:x}'.format(crc_l)]        
    #return "b'" + message + '\\' + str(hex(crc_h)) + '\\' + str(hex(crc_l)) + "'"
    #return "b'"+ str(message)[:-2] + '\\' + str(hex(crc_h)[1:]) + '\\' + str(hex(crc_l)[1:])+ "\\x0d'"
    #return str(message)[:-2] + '\\' + str(hex(crc_h)[1:]) + '\\' + str(hex(crc_l)[1:])+ '\\x0d'
    #return str(message)[:-2], hex(crc_h), hex(crc_l), '\r'

while True:

    message = input('>>')
    CRC16 = str_CRC16(message)
    #crc = checkCRC(message)
    #crc_h = crc / 0x100
    #crc_l = crc & 0xFF
    #out = '\\' + str(hex(crc_h))[1:] + '\\' + str(hex(crc_l))[1:]
    #print(str_CRC16(message)+'\r')
    # print('{:x}, {:x}'.format(*CRC16))
       
    print(message + '\x00\x00')
    print(CRC16)
    #print((CRC16).hex())
    
    #print(bytes((message).encode('utf-8'))+CRC16, CRC16)
    
