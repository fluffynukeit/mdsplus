from MDSplus import *
import serial
import time
import socket
import sys


class ARDUINO_MUX(Device):
    parts = [{'path': ':COMMENT', 'type': 'text'},
             {'path': ':ARDUINO_IP', 'type': 'text', 'value': '192.168.1.77'},
             {'path': ':ARDUINO_PORT', 'type': 'text', 'value': '/dev/ttyACM0'},
             {'path': ':MODE', 'type': 'text', 'value': 'SERIAL'},
             {'path': '.MUX1', 'type': 'structure'},
             {'path': '.MUX1:READ_BACK',  'type': 'text', 'value': 'NO'},
             {'path': '.MUX1:WLINE1',  'type': 'numeric', 'value': 0},
             {'path': '.MUX1:WLINE2',  'type': 'numeric', 'value': 1},
             {'path': '.MUX1:WLINE3',  'type': 'numeric', 'value': 2},
             {'path': '.MUX1:RLINE1',  'type': 'numeric', 'value': 0},
             {'path': '.MUX1:RLINE2',  'type': 'numeric', 'value': 0},
             {'path': '.MUX1:RLINE3',  'type': 'numeric', 'value': 0},
             {'path': '.MUX1:SELECTED',  'type': 'numeric', 'value': 0},
             {'path': '.MUX1:SEL_READ',  'type': 'numeric'},
             {'path': '.MUX2', 'type': 'structure'},
             {'path': '.MUX2:READ_BACK',  'type': 'text', 'value': 'NO'},
             {'path': '.MUX2:WLINE1',  'type': 'numeric', 'value': 3},
             {'path': '.MUX2:WLINE2',  'type': 'numeric', 'value': 4},
             {'path': '.MUX2:WLINE3',  'type': 'numeric', 'value': 5},
             {'path': '.MUX2:RLINE1',  'type': 'numeric', 'value': 0},
             {'path': '.MUX2:RLINE2',  'type': 'numeric', 'value': 0},
             {'path': '.MUX2:RLINE3',  'type': 'numeric', 'value': 0},
             {'path': '.MUX2:SELECTED',  'type': 'numeric', 'value': 0},
             {'path': '.MUX2:SEL_READ',  'type': 'numeric'},
             {'path': ':INIT_ACTION', 'type': 'action',
              'valueExpr': "Action(Dispatch('ARDUINO_SERVER','PULSE_PREPARATION',50,None),Method(None,'init',head))",
              'options': ('no_write_shot',)}]

    def doSerialCommand(self, ser, cmd):
        ser.write(bytes((cmd + '\n').encode('utf-8')))
        print(ser.readline())
        res = ser.readline()
        print(res)
        time.sleep(0.1)
        return res
 
    def doTcpCommand(self, sock, cmd):
        sock.sendall(bytes((cmd + '\n').encode('utf-8')))
        resStr = ''
        while True:
            data = sock.recv(1)
            if data.decode('utf-8') == '\n':
                break
            resStr += data.decode('utf-8')
        print(resStr)
        resStr = ''
        while True:
            data = sock.recv(1)
            if data.decode('utf-8') == '\n':
                break
            resStr += data.decode('utf-8')
        print(resStr)
        time.sleep(0.1)
        return resStr
        
  
    def init(self):
        print('================= ARDUINO_MUX Init ===============')
        if self.mode.data() == 'SERIAL':
            isSerial = True
            maxDigital = 13
        else:
            isSerial = False;
            maxDigital = 9
        outLines = []   
        inLines = []   
        if isSerial:
            try:
                serName = self.arduino_port.data()
            except:
                print('Serial port not defined')
                raise mdsExceptions.TclFAILED_ESSENTIAL
            try:
                ser = serial.Serial(port=serName, baudrate = 9600, timeout = .1)
                time.sleep(.5)
                ser.write(bytes('\n'.encode('utf-8')))
            except:
                print('Cannot open serial port '+serName)
                raise mdsExceptions.TclFAILED_ESSENTIAL
        else:
            try:
                ipAddress = self.ip.data()
            except:
                print('IP address not defined')
                raise mdsExceptions.TclFAILED_ESSENTIAL
            try:
                 sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                 sock.setsockopt( socket.IPPROTO_TCP, socket.TCP_NODELAY, 1 )
                 server_address = (ipAddress, 23)
                 print('connecting ...')
                 sock.connect(server_address)
            except:
                print('Cannot connect to '+ ipAddress)
                raise mdsExceptions.TclFAILED_ESSENTIAL

        commands = ['PRGSTART']
        for mpx in range(2):
            line1 = getattr(self, 'mux%d_wline1' % (mpx + 1)).data()
            line2 = getattr(self, 'mux%d_wline2' % (mpx + 1)).data()
            line3 = getattr(self, 'mux%d_wline2' % (mpx + 1)).data()
            if line1 < 0 or line1 > maxDigital or line2 < 0 or line2 > maxDigital or line3 < 0 or line3 > maxDigital:
                print('Only digital lines between 0 and '+maxDigital+' can be used')
                raise mdsExceptions.TclFAILED_ESSENTIAL
            if line1 in inLines or line2 in inLines or line3 in inLines:
                print('Same line used as input and output')
                raise mdsExceptions.TclFAILED_ESSENTIAL
            outLines.append(line1)
            outLines.append(line2)
            outLines.append(line3)
               
            mask = (1 << line1)|(1 << line2)|(1 << line3)
            maskStr = format(mask, '04X')
            for m in range(8):
                mm = m
                bit1 = mm % 2
                mm /= 2
                bit2 = mm % 2
                mm /= 2
                bit3 = mm % 2
                bits = (bit1 << line1)|(bit2 << line2) | (bit3 << line3)
                bitsStr = format(bits, '04X')
                currCommand = 'WM'+str(mpx+1)+str(m)+maskStr+bitsStr+'CM'+str(mpx+1)+format(m, '02X')
                commands.append(currCommand)
            if getattr(self, 'mux%d_read_back' % (mpx + 1)).data() == 'YES':
                if line1 < 0 or line1 > maxDigital or line2 < 0 or line2 > maxDigital or line3 < 0 or line3 > maxDigital:
                    print('Only digital lines between 0 and '+ maxDigital + ' can be used')
                    raise mdsExceptions.TclFAILED_ESSENTIAL
                line1 = getattr(self, 'mux%d_rline1' % (mpx + 1)).data()
                line2 = getattr(self, 'mux%d_rline2' % (mpx + 1)).data()
                line3 = getattr(self, 'mux%d_rline3' % (mpx + 1)).data()
                if line1 in outLines or line2 in outLines or line3 in outLines:
                    print('Same line used as input and output')
                    raise mdsExceptions.TclFAILED_ESSENTIAL
                inLines.append(line1)
                inLines.append(line2)
                inLines.append(line3)
                mask = (1 << line1)|(1 << line2)|(1 << line3)
                maskStr = format(mask, '04X')
                for m in range(8):
                    mm = m
                    bit1 = mm % 2
                    mm /= 2
                    bit2 = mm % 2
                    mm /= 2
                    bit3 = mm % 2
                    bits = (bit1 << line1)|(bit2 << line2) | (bit3 << line3)
                    bitsStr = format(bits, '04X')
                    currCommand = 'RRM'+str(mpx+1)+maskStr+bitsStr+format(m, '02X')
                    commands.append(currCommand)
        commands.append('PRGSTOP')
        for command in commands:
            if isSerial:
                self.doSerialCommand(ser, command)
            else:
                self.doTcpCommand(sock, command)
        selected = self.mux1_selected.data()
        command = 'M1'+str(selected)
        
        if isSerial:
            ans = self.doSerialCommand(ser, command)
        else:
            ans = self.doTcpCommand(ser, command)
        if self.mux1_read_back.data() == 'YES':
            command = 'RM1'
            if isSerial:
                ans = self.doSerialCommand(ser, command)
            else:
                ans = self.doTcpCommand(ser, command)
            self.mux1_sel_read.putData(int(ans))
          
          
        selected = self.mux2_selected.data()
        command = 'M2'+str(selected)
        
        if isSerial:
            ans = self.doSerialCommand(ser, command)
        else:
            ans = self.doTcpCommand(ser, command)
        if self.mux2_read_back.data() == 'YES':
            command = 'RM2'
            if isSerial:
                ans = self.doSerialCommand(ser, command)
            else:
                ans = self.doTcpCommand(ser, command)
            self.mux2_sel_read.putData(int(ans))
        
