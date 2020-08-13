import serial
import time
import sys
import struct
aaa = []
aaa=[
	0X00,0X07,0X0E,0X09,0X1C,0X1B,0X12,0X15,0X38,0X3F,0X36,0X31,0X24,0X23,0X2A,0X2D,
	0X70,0X77,0X7E,0X79,0X6C,0X6B,0X62,0X65,0X48,0X4F,0X46,0X41,0X54,0X53,0X5A,0X5D,
	0XE0,0XE7,0XEE,0XE9,0XFC,0XFB,0XF2,0XF5,0XD8,0XDF,0XD6,0XD1,0XC4,0XC3,0XCA,0XCD,
	0X90,0X97,0X9E,0X99,0X8C,0X8B,0X82,0X85,0XA8,0XAF,0XA6,0XA1,0XB4,0XB3,0XBA,0XBD,
	0XC7,0XC0,0XC9,0XCE,0XDB,0XDC,0XD5,0XD2,0XFF,0XF8,0XF1,0XF6,0XE3,0XE4,0XED,0XEA,
	0XB7,0XB0,0XB9,0XBE,0XAB,0XAC,0XA5,0XA2,0X8F,0X88,0X81,0X86,0X93,0X94,0X9D,0X9A,
	0X27,0X20,0X29,0X2E,0X3B,0X3C,0X35,0X32,0X1F,0X18,0X11,0X16,0X03,0X04,0X0D,0X0A,
	0X57,0X50,0X59,0X5E,0X4B,0X4C,0X45,0X42,0X6F,0X68,0X61,0X66,0X73,0X74,0X7D,0X7A,
	0X89,0X8E,0X87,0X80,0X95,0X92,0X9B,0X9C,0XB1,0XB6,0XBF,0XB8,0XAD,0XAA,0XA3,0XA4,
	0XF9,0XFE,0XF7,0XF0,0XE5,0XE2,0XEB,0XEC,0XC1,0XC6,0XCF,0XC8,0XDD,0XDA,0XD3,0XD4,
	0X69,0X6E,0X67,0X60,0X75,0X72,0X7B,0X7C,0X51,0X56,0X5F,0X58,0X4D,0X4A,0X43,0X44,
	0X19,0X1E,0X17,0X10,0X05,0X02,0X0B,0X0C,0X21,0X26,0X2F,0X28,0X3D,0X3A,0X33,0X34,
	0X4E,0X49,0X40,0X47,0X52,0X55,0X5C,0X5B,0X76,0X71,0X78,0X7F,0X6A,0X6D,0X64,0X63,
	0X3E,0X39,0X30,0X37,0X02,0X25,0X2C,0X2B,0X06,0X01,0X08,0X0F,0X1A,0X1D,0X14,0X13,
	0XAE,0XA9,0XA0,0XA7,0XB2,0XB5,0XBC,0XBB,0X96,0X91,0X98,0X9F,0X8A,0X8D,0X84,0X83,
	0XDE,0XD9,0XD0,0XD7,0XC2,0XC5,0XCC,0XCB,0XE6,0XE1,0XE8,0XEF,0XFA,0XFD,0XF4,0XF3]
ser = serial.Serial(
	port = '/dev/ttyS0',
	baudrate = 115200,
	parity = serial.PARITY_NONE,
	stopbits = serial.STOPBITS_ONE,
	bytesize = serial.EIGHTBITS,
	timeout = 0.01
)
#data = 'TH\t2222\t2222\t2222\t2222\t2222\t2222\t2222\t2222\r\n'
#print 'asdsad'
def calCRC(p):
	i = 0
	j = 0
	crc = 0
	leng = len(p)
	while (leng):
		i = (crc ^ p[j]) & 0X00FF
		j+=1
		crc = (aaa[i] ^ (crc<<8)) & 0x00FF
		leng -=1 
	return (crc & 0X00FF)
data = []
for i in range(19):
	data.append(0)
data[0] = 84
data[1] = 72
k = 999
try:
	while True:
		while k<6000:
			for i in range(8):
				data[(i+1)*2] = (k >> 8) & 0XFF
				data[(i+1)*2 + 1] = (k & 0XFF)
			data[18] = calCRC(data[0:18])
			ser.write(bytearray(data))
			
			k = k + 1
			time.sleep(0.01)
			print k
		k = 0
except KeyboardInterrupt:
	ser.close()

#data.append(0X09)

#for i in range(8):
	#data.append(0X27)
	#data.append(0X10)

#data.append(calCRC(data))
#data.append(0)
print (data)
print (sys.getsizeof(0))
print (b'aaa')
#print (data[1].to_bytes(2,byteorder = 'big'))
#print ((1111).to_bytes(2,byteorder = 'big'))
#(2221).to_bytes(10, byteorder='big')
#time.sleep(2)

#print sys.getsizeof(struct.pack("<H",0X90))
#print sys.getsizeof(0X90)
print (sys.getsizeof(str(82)))
#print (bytes(data))
#time.sleep(2)

#try: 
#	while True:
		#ser.write(bytes(data))
		#ser.write(84)
		#ser.write(72)
		#for i in range(8):
		#	ser.write(1)
		#	ser.write(144)
		#ser.write(11)
#		ser.write(bytearray(data))
			#print (data[i].to_bytes(1,byteorder = 'big'))
			#ser.write(str(data[i]).encode())
			#ser.write(bytes(data[i]).encode())
			
			#ser.write(struct.pack(">H",data[i]))
			#print bytes(data[i]).encode()
			#print (struct.pack(">h",data[i]))
			#ser.write(bytes(data[i]))
		#time.sleep(0.001)
		#ser.write(data[0])
		#print(calCRC(data))
#		print 'writing to AP'
#		time.sleep(0.01)
		#print data
#except KeyboardInterrupt:
#	ser.close()
