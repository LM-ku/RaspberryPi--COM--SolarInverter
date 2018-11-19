import serial

pipInputBuf = [""] * 500
pipInputPointer = 0
reseiving_data = ""
sending_data = ""
length = 0



# Свойство Serial.in_waiting возвращает "количество байтов в буфере приема".
# Это, по-видимому, эквивалентно Serial.available()
# описание: "количество байтов..., которые уже были получены и сохранены в последовательном буфера приема".

# Возможные значения timeout :
# 1. Нет: ждать вечно, блокировать вызов
# 2. 0: non-преграждая режим, возвращает немедленно
# 3. x, x больше 0, float разрешен, вызов блока тайм-аута




# ИНИЦИАЛИЗАЦИЯ ПОСЛЕДОВАТЕЛЬНОГО ПОРТА

ser = serial.Serial( '/dev/ttyAMA0', 2400,
                     parity=serial.PARITY_NONE,
                     stopbits=serial.STOPBITS_ONE,
                     bytesize=serial.EIGHTBITS,
)

ser.open()
ser.isOpen()
    

# Structure to store the data from the PIP4048

pipVals = {gridVoltage[16], gridFrequency[16], acOutput[16], acFrequency[16], acApparentPower[16],
           acActivePower[16], loadPercent[16], busVoltage[16], batteryVoltage[16], batteryChargeCurrent[16],
           batteryCharge[16], inverterTemperature[16], PVCurrent[16], PVVoltage[16]
}

# Some useful PIP commands

pipCommands = {QPIGS : {0x51,0x50,0x49,0x47,0x53,0xB7,0xA9,0x0d},
               QPIWS : {0x51,0x50,0x49,0x57,0x53,0xB4,0xDA,0x0D}
} 

whichPIPCommand = 0
lastPIPCommand = 0








# loop ()

#crc_l = 0
#crc_h = 0
#*val;
#char pipstatus[40];
#float power;


# ЧТЕНИЕ ИЗ ИНВЕРТОРА ДАННЫХ
#

def processPipInput(retCRC):
    newCrc = 0

    if ser.isOpen():
        pipChar = Ser.read()
        if pipChar() != 0x0d:
            pipInputPointer += 1
            pipInputBuf[pipInputPointer] = pipChar
		
	else:
            newCRC = cal_crc_half(pipInputBuf, pipInputPointer - 2)
            if (newCRC == ((((pipInputBuf[pipInputPointer - 2]) << 8) & 0xff00)	| (pipInputBuf[pipInputPointer - 1] & 0xff))) : # СRC is good !
                length = pipInputPointer - 2
		pipInputBuf[i] = 0                      # Terminate the string in the input buffer, overwriting the crc - so it can  easily be printed out
		pipInputPointer = 0                     # Zero the pointer ready for the next packet
		retCRC = newCRC                         # Return the buffer CRC
		return(length)                          # Return length of buffer
			
	    else:
                pipInputBuf[pipInputPointer + 1] = 0    # Terminate the string for display...keep the crc in place for checking
		pipInputPointer = 0
		return (-1)                             # Indicate bad crc
			
    return (0)                                          # packet not yet finished




delay(5000)     # read every 5 seconds






# Send alternating commands to PIP - QPIGS then QPIWS
# Commands with CRC cheats
# QPGS0 = '\x51\x50\x47\x53\x30\x3f\xda\x0d'
# QPIGS = '\x51\x50\x49\x47\x53\xB7\xA9\x0d'
# QMCHGCR ='\x51\x4D\x43\x48\x47\x43\x52\xD8\x55\x0D' #?
# QMUCHGCR='\x51\x4D\x55\x43\x48\x47\x43\x52\x26\x34\x0D' #?
# QPIWS = '\x51\x50\x49\x57\x53\xB4\xDA\x0D' #valid?
# POP02 = '\x50\x4F\x50\x30\x32\xE2\x0B\x0D' # set to SBU
# POP00 = '\x50\x4F\x50\x30\x30\xC2\x48\x0D' #Set to UTILITY
# "QMOD\x49\xC1";
# "QID\xD6\xEA";
# "QVFW\x62\x99";
# "QVFW2\xC3\xF5";
# "QPIRI\xF8\x54";   -->51 50 49 52 49
# QPIRI = '\x51\x50\x49\x52\x49\xF8\x54\x0D'
# "QFLAG\x98\x74";










# ПООЧЕРЕДНЫЙ ЗАПРОС ОТ ИНВЕРТОРА ПАРАМЕТРОВ И ОШИБОК

if whichPIPCommand == 0 :
    sending_data = bytearray(QPIGS)
    lastPIPCommand = 0
    whichPIPCommand = 1
    
if whichPIPCommand == 1 :
    sending_data = bytearray(QPIWS)
    lastPIPCommand = 1
    whichPIPCommand = 0

ser.write(sending_data)
        






# Check any return from the pip4048

i = processPipInput(&crc);
  if (i > 0) // Got a good packet
  	{
  	mySerial.println("Got good packet");
  	mySerial.println((char *) pipInputBuf);

  	switch (lastPIPCommand) // Which paccket are we expecting?
  		{
  		case 0: // QPIGS
  		// Now split the packet into the values
  		val = strtok((char *) pipInputBuf, " "); // get the first value
  		strcpy(pipVals.gridVoltage, val + 1); // Skip the initial '('

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.gridFrequency, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.acOutput, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.acFrequency, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.acApparentPower, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.acActivePower, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.loadPercent, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.busVoltage, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.batteryVoltage, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.batteryChargeCurrent, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.batteryCharge, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.inverterTemperature, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.PVCurrent, val);

  		val = strtok(0, " "); // Get the next value
  		strcpy(pipVals.PVVoltage, val);


# This is to send to an MQTT server
  		client.publish(catString(outTopic, "/gridVoltage"), pipVals.gridVoltage);
  		client.publish(catString(outTopic, "/gridFrequency"), pipVals.gridFrequency);
  		client.publish(catString(outTopic, "/acOutput"), pipVals.acOutput);
  		client.publish(catString(outTopic, "/acFrequency"), pipVals.acFrequency);
  		client.publish(catString(outTopic, "/acApparentPower"), pipVals.acApparentPower);
  		client.publish(catString(outTopic, "/acActivePower"), pipVals.acActivePower);
  		client.publish(catString(outTopic, "/loadPercent"), pipVals.loadPercent);
  		client.publish(catString(outTopic, "/busVoltage"), pipVals.busVoltage);
  		client.publish(catString(outTopic, "/batteryVoltage"), pipVals.batteryVoltage);
  		client.publish(catString(outTopic, "/batteryChargeCurrent"), pipVals.batteryChargeCurrent);
  		client.publish(catString(outTopic, "/batteryCharge"), pipVals.batteryCharge);
  		client.publish(catString(outTopic, "/inverterTemperature"), pipVals.inverterTemperature);
  		client.publish(catString(outTopic, "/PVCurrent"), pipVals.PVCurrent);
  		client.publish(catString(outTopic, "/PVVoltage"), pipVals.PVVoltage);

# Calculate PV Power
  		int I, V;
  		I = atoi(pipVals.PVCurrent);
  		V = atoi(pipVals.PVVoltage);
  		sprintf(s, "%d", I * V);
  		client.publish(catString(outTopic, "/PVPower"), s);

  		break;

  		case 1: // QPIWS
  		val = strtok((char *) pipInputBuf, " "); // get the first value
  		strcpy(pipstatus, val + 1); // Skip the initial '(' - make a copy of the returned stricg for processing

  		// Now send the various PIP status messages - CONVERT TO JSON FOR USE
  		if (pipstatus[1] == '1')
  			client.publish(catString(outTopic, "/status"), "Inverter Fault");

  		if (pipstatus[2] == '1')
  			client.publish(catString(outTopic, "/status"), "Bus Over Voltage");

  		if (pipstatus[3] == '1')
  			client.publish(catString(outTopic, "/status"), "Bus Under Voltage");

  		if (pipstatus[4] == '1')
  			client.publish(catString(outTopic, "/status"), "Bus Soft Fail");

  		if (pipstatus[5] == '1')
  			client.publish(catString(outTopic, "/status"), "Line Fail");

  		if (pipstatus[6] == '1')
  			client.publish(catString(outTopic, "/status"), "OPV Short");

  		if (pipstatus[7] == '1')
  			client.publish(catString(outTopic, "/status"), "Inverter Voltage Too Low");

  		if (pipstatus[8] == '1')
  			client.publish(catString(outTopic, "/status"), "Inverter Voltage Too High");

  		if (pipstatus[9] == '1')
  			client.publish(catString(outTopic, "/status"), "Over Temperature");

  		if (pipstatus[10] == '1')
  			client.publish(catString(outTopic, "/status"), "Fan Locked");

  		if (pipstatus[11] == '1')
  			client.publish(catString(outTopic, "/status"), "Battery Voltage Too High");

  		if (pipstatus[12] == '1')
  			client.publish(catString(outTopic, "/status"), "Battery Low Alarm");

  		if (pipstatus[14] == '1')
  			client.publish(catString(outTopic, "/status"), "Battery Under Shutdown");

  		if ((pipstatus[16] == '1') && (pipstatus[1] == '0'))
  			client.publish(catString(outTopic, "/status"), "Overload - Warning");

  		if ((pipstatus[16] == '1') && (pipstatus[1] == '1'))
  			client.publish(catString(outTopic, "/status"), "Overload - FAULT");

  		if (pipstatus[17] == '1')
  			client.publish(catString(outTopic, "/status"), "EEPROM Fault");

  		break;
  		}
  	}

  if (i == -1) // Got a bad packet
  	{
  	mySerial.println("Got BAD packet");
  	mySerial.println((char *) pipInputBuf);
  	}


}


// Check for input from serial1, put it into a buffer and then return the buffer length if a <cr> has been detected and the packet is valid,
// 0 if <cr> hasn't yet been detected and -1 if an invalid crc has been sent

int processPipInput(uint16_t *retCrc)
{
uint8_t pipChar;
uint16_t newCrc;

while (Serial1.available()) // Got any input?
	{
	if ((pipChar = Serial1.read()) != 0x0d) // Read the byte
		{
		pipInputBuf[pipInputPointer++] = pipChar; // Not a <cr>
		}
	else
		{ // Got a <cr>, calculate the crc
		newCrc = cal_crc_half(pipInputBuf, pipInputPointer - 2);
		if (newCrc == ((((pipInputBuf[pipInputPointer - 2]) << 8) & 0xff00)	| (pipInputBuf[pipInputPointer - 1] & 0xff))) // Good crc
			{
			int i = pipInputPointer - 2;
			pipInputBuf[i] = 0; // Terminate the string in the input buffer, overwriting the crc - so it can  easily be printed out
			pipInputPointer = 0; // Zero the pointer ready for the next packet
			*retCrc = newCrc; // Return the buffer CRC
			return(i); // Return length of buffer
			}
		else
			{
			pipInputBuf[pipInputPointer + 1] = 0; // Terminate the string for display...keep the crc in place for checking
			pipInputPointer = 0;
			return (-1); // Indicate bad crc
			}
		}
	}

return (0); // packet not yet finished
}


// Send a packet to the pip4048

void pipSend(uint8_t txArray[], int length)
{
int crc = cal_crc_half(txArray, length);

Serial1.write(txArray, length);
Serial1.write((crc >> 8) & 0xff);
Serial1.write(crc & 0xff);
Serial1.write(0x0d);
}


uint16_t cal_crc_half(uint8_t  *pin, uint8_t len)
{

	uint16_t crc;

	uint8_t da;
	uint8_t  *ptr;
	uint8_t bCRCHign;
	uint8_t bCRCLow;

	uint16_t crc_ta[16]=
	{
		0x0000,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,

		0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef
	};
	ptr=pin;
	crc=0;

	while(len--!=0)
	{
		da=((uint8_t)(crc>>8))>>4;

		crc<<=4;

		crc^=crc_ta[da^(*ptr>>4)];

		da=((uint8_t)(crc>>8))>>4;

		crc<<=4;

		crc^=crc_ta[da^(*ptr&0x0f)];

		ptr++;
	}
	bCRCLow = crc;

	bCRCHign= (uint8_t)(crc>>8);

	if(bCRCLow==0x28||bCRCLow==0x0d||bCRCLow==0x0a)

	{
		bCRCLow++;
	}
	if(bCRCHign==0x28||bCRCHign==0x0d||bCRCHign==0x0a)

	{
		bCRCHign++;
	}
	crc = ((uint8_t)bCRCHign)<<8;
	crc += bCRCLow;
	return(crc);
}

// Concatenate 2 strings - max result is 256 bytes
char * catString (char *s1, char *s2)
{
	static char result[256];
	sprintf(result, "%s%s", s1, s2);
	return (result);
}
