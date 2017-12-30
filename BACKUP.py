'''
	Original by dzhu
		https://github.com/dzhu/myo-raw

	Edited by Fernando Cosentino
		http://www.fernandocosentino.net/pyoconnect
'''


from __future__ import print_function

import enum
import re
import struct
import sys
import threading
import time
from threading import Lock
import thread
import serial
from serial.tools.list_ports import comports
#import freqshow
from common import *
### START TEST
import os
import time

import pygame

import controller
import model
import ui

import views

#import myo_raw
###

def multichr(ords):
    if sys.version_info[0] >= 3:
        return bytes(ords)
    else:
        return ''.join(map(chr, ords))

def multiord(b):
    if sys.version_info[0] >= 3:
        return list(b)
    else:
        return map(ord, b)

class Arm(enum.Enum):
    UNKNOWN = 0
    RIGHT = 1
    LEFT = 2

class XDirection(enum.Enum):
    UNKNOWN = 0
    X_TOWARD_WRIST = 1
    X_TOWARD_ELBOW = 2

class Pose(enum.Enum):
    REST = 0
    FIST = 1
    WAVE_IN = 2
    WAVE_OUT = 3
    FINGERS_SPREAD = 4
    THUMB_TO_PINKY = 5
    UNKNOWN = 255	 
    
class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % \
            (self.typ, self.cls, self.cmd,
             ' '.join('%02X' % b for b in multiord(self.payload)))


class BT(object):
    '''Implements the non-Myo-specific details of the Bluetooth protocol.'''
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
        self.buf = []
        self.lock = threading.Lock()
        self.handlers = []

    ## internal data-handling methods
    def recv_packet(self, timeout=None):
       # print ("got packet")
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None: self.ser.timeout = t0 + timeout - time.time()
            c = self.ser.read()
            if not c: return None

            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)
                return ret

    def recv_packets(self, timeout=.5):
        res = []
        t0 = time.time()
        while time.time() < t0 + timeout:
            p = self.recv_packet(t0 + timeout - time.time())
            if not p: return res
            res.append(p)
        return res

    def proc_byte(self, c):
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try: self.handlers.remove(h)
        except ValueError: pass

    def wait_event(self, cls, cmd):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        while res[0] is None:
            self.recv_packet()
        self.remove_handler(h)
        return res[0]

    ## specific BLE commands
    def connect(self, addr):
        return self.send_command(6, 3, pack('6sBHHHH', multichr(addr), 0, 6, 6, 64, 0))

    def get_connections(self):
        return self.send_command(0, 6)

    def discover(self):
        return self.send_command(6, 2, b'\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload=b'', wait_resp=True):
        s = pack('4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)

        while True:
            p = self.recv_packet()

            ## no timeout, so p won't be None
            if p.typ == 0: return p

            ## not a response: must be an event
            self.handle_event(p)


class MyoRaw(threading.Thread):
    '''Implements the Myo-specific communication protocol.'''

    def __init__(self, tty=None):
        threading.Thread.__init__(self)
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')

        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.imu_handlers = []
        self.arm_handlers = []
        self.pose_handlers = []
        self.running = False

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('using myo:', p[0])
                return p[0]

        return None


    def run(self):
        self.running = True
        while(self.running):
            time.sleep(1000)
            #act()

    def end(self):
        self.running = False
    def act(self):
        self.bt.recv_packet(1)


    def connect(self):
        ## stop everything from before
        self.bt.end_scan()
        self.bt.disconnect(0)
        self.bt.disconnect(1)
        self.bt.disconnect(2)

        ## start scanning
        print('scanning...')
        self.bt.discover()
        while True:
            p = self.bt.recv_packet()
            print('scan response:', p)

            if p.payload.endswith(b'\x06\x42\x48\x12\x4A\x7F\x2C\x48\x47\xB9\xDE\x04\xA9\x01\x00\x06\xD5'):
                addr = list(multiord(p.payload[2:8]))
                break
        self.bt.end_scan()

        ## connect and wait for status event
        conn_pkt = self.bt.connect(addr)
        self.conn = multiord(conn_pkt.payload)[-1]
        self.bt.wait_event(3, 0)

        ## get firmware version
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('BHBBHHHH', fw.payload)
        print('firmware version: %d.%d.%d.%d' % (v0, v1, v2, v3))

        self.old = (v0 == 0)

        if self.old:
            ## don't know what these do; Myo Connect sends them, though we get data
            ## fine without them
            self.write_attr(0x19, b'\x01\x02\x00\x00')
            self.write_attr(0x2f, b'\x01\x00')
            self.write_attr(0x2c, b'\x01\x00')
            self.write_attr(0x32, b'\x01\x00')
            self.write_attr(0x35, b'\x01\x00')

            ## enable EMG data
            self.write_attr(0x28, b'\x01\x00')
            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')

            ## Sampling rate of the underlying EMG sensor, capped to 1000. If it's
            ## less than 1000, emg_hz is correct. If it is greater, the actual
            ## framerate starts dropping inversely. Also, if this is much less than
            ## 1000, EMG data becomes slower to respond to changes. In conclusion,
            ## 1000 is probably a good value.
            C = 1000
            emg_hz = 50
            ## strength of low-pass filtering of EMG data
            emg_smooth = 100

            imu_hz = 50

            ## send sensor parameters, or we don't get any data
            self.write_attr(0x19, pack('BBBBHBBBBB', 2, 9, 2, 1, C, emg_smooth, C // emg_hz, imu_hz, 0, 0))

        else:
            name = self.read_attr(0x03)
            print('device name: %s' % name.payload)

            ## enable IMU data
            self.write_attr(0x1d, b'\x01\x00')
            ## enable on/off arm notifications
            self.write_attr(0x24, b'\x02\x00')

            # self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
            self.start_raw()

        ## add data handlers
        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5): return

            c, attr, typ = unpack('BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('8HB', pay)
                ## not entirely sure what the last byte is, but it's a bitmask that
                ## seems to indicate which sensors think they're being moved around or
                ## something
                emg = vals[:8]
                moving = vals[8]
                self.on_emg(emg, moving)
            elif attr == 0x1c:
                vals = unpack('10h', pay)
                quat = vals[:4]
                acc = vals[4:7]
                gyro = vals[7:10]
                self.on_imu(quat, acc, gyro)
            elif attr == 0x23:
                typ, val, xdir, _,_,_ = unpack('6B', pay)

                if typ == 1: # on arm
                    self.on_arm(Arm(val), XDirection(xdir))
                elif typ == 2: # removed from arm
                    self.on_arm(Arm.UNKNOWN, XDirection.UNKNOWN)
                elif typ == 3: # pose
                    if (Pose(val) == Pose.FINGERS_SPREAD):
                        key = [0,999,0,999,0,999,0,999,0,999,0,999]
                        self.on_emg(key,key)
                    self.on_pose(Pose(val))
            else:
                print('data with unknown attr: %02X %s' % (attr, p))

        self.bt.add_handler(handle_data)


    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def read_attr(self, attr):
        if self.conn is not None:
            return self.bt.read_attr(self.conn, attr)
        return None

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    def start_raw(self):
        '''Sending this sequence for v1.0 firmware seems to enable both raw data and
        pose notifications.
        '''

        self.write_attr(0x28, b'\x01\x00')
        #self.write_attr(0x19, b'\x01\x03\x01\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def mc_start_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when starting data
        collection for v1.0 firmware; this enables raw data but disables arm and
        pose notifications.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x09\x01\x01\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x00')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x00')

    def mc_end_collection(self):
        '''Myo Connect sends this sequence (or a reordering) when ending data collection
        for v1.0 firmware; this reenables arm and pose notifications, but
        doesn't disable raw data.
        '''

        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')
        self.write_attr(0x19, b'\x09\x01\x00\x00\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x00\x01\x01')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x1d, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

    def vibrate(self, length):
        if length in xrange(1, 4):
            ## first byte tells it to vibrate; purpose of second byte is unknown
            self.write_attr(0x19, pack('3B', 3, 1, length))


    def add_emg_handler(self, h):
        self.emg_handlers.append(h)

    def add_imu_handler(self, h):
        self.imu_handlers.append(h)

    def add_pose_handler(self, h):
        self.pose_handlers.append(h)

    def add_arm_handler(self, h):
        self.arm_handlers.append(h)


    def on_emg(self, emg, moving):
        for h in self.emg_handlers:
            h(emg, moving)

    def on_imu(self, quat, acc, gyro):
        for h in self.imu_handlers:
            h(quat, acc, gyro)

    def on_pose(self, p):
        for h in self.pose_handlers:
            h(p)

    def on_arm(self, arm, xdir):
        for h in self.arm_handlers:
            h(arm, xdir)


if __name__ == '__main__':
    SDR_SAMPLE_SIZE = 1024  # Number of samples to grab from the radio.  Should be
                        # larger than the maximum display width.

    CLICK_DEBOUNCE  = 0.4   # Number of seconds to wait between clicks events. Set
                            # to a few hunded milliseconds to prevent accidental
                            # double clicks from hard screen presses.

    # Font size configuration.
    MAIN_FONT = 33
    NUM_FONT  = 50

    # Color configuration (RGB tuples, 0 to 255).
    MAIN_BG        = (  0,   0,   0) # Black
    INPUT_BG       = ( 60, 255, 255) # Cyan-ish
    INPUT_FG       = (  0,   0,   0) # Black
    CANCEL_BG      = (128,  45,  45) # Dark red
    ACCEPT_BG      = ( 45, 128,  45) # Dark green
    BUTTON_BG      = ( 60,  60,  60) # Dark gray
    BUTTON_FG      = (255, 255, 255) # White
    BUTTON_BORDER  = (200, 200, 200) # White/light gray
    INSTANT_LINE   = (  0, 255, 128) # Bright yellow green.

    # Define gradient of colors for the waterfall graph.  Gradient goes from blue to
    # yellow to cyan to red.
    WATERFALL_GRAD = [(0, 0, 255), (0, 255, 255), (255, 255, 0), (255, 0, 0)]

    # Configure default UI and button values.
    ui.MAIN_FONT = MAIN_FONT
    ui.Button.fg_color     = BUTTON_FG
    ui.Button.bg_color     = BUTTON_BG
    ui.Button.border_color = BUTTON_BORDER
    ui.Button.padding_px   = 2
    ui.Button.border_px    = 2
    fsmodel = model.FreqShowModel(1920, 1080)
    ###


    try:
        import pygame
        from pygame.locals import *
        HAVE_PYGAME = False
    except ImportError:
    	print("NOGAME")
        HAVE_PYGAME = False
		
    if HAVE_PYGAME:
        w, h = 1200, 400
        scr = pygame.display.set_mode((w, h))

    last_vals = None
    
    def plot(scr, vals):
        DRAW_LINES = False

        global last_vals
        if last_vals is None:
            last_vals = vals
            return

        D = 5
        scr.scroll(-D)
        scr.fill((0,0,0), (w - D, 0, w, h))
        for i, (u, v) in enumerate(zip(last_vals, vals)):
            if DRAW_LINES:
                pygame.draw.line(scr, (0,255,0),
                                 (w - D, int(h/8 * (i+1 - u))),
                                 (w, int(h/8 * (i+1 - v))))
                pygame.draw.line(scr, (255,255,255),
                                 (w - D, int(h/8 * (i+1))),
                                 (w, int(h/8 * (i+1))))
            else:
                c = int(255 * max(0, min(1, v)))
                scr.fill((c, c, c), (w - D, i * h / 8, D, (i + 1) * h / 8 - i * h / 8));

        pygame.display.flip()
        last_vals = vals

    m = MyoRaw(sys.argv[1] if len(sys.argv) >= 2 else None)
    last = [False, False, False, False, False, False, False, False, False, False]

    lock = Lock()
    def proc_emg(emg, moving, times=[]):

        ## update pygame display
        #plot(scr, [e / 2000. for e in emg])
        print(emg)
    
        
        if (emg == [0,999,0,999,0,999,0,999,0,999,0,999]):
            print ("DOWN")
            #freqshow.update_center(-1)
        elif ((True in last) and (emg[5]> 800 and emg[6] > 800) and (up==False)):
            print ("UP")
            
        
        elif (max(emg[1:]) == emg[2] and emg[2]>100):# and emg[1]+emg[3] < 200):
           # freqshow.update_center(1)
            #print("HAND" + str(last))
            if (last==[False, False, False, False, False, False, False, False, False, False]):
                #hand = True
                if (not lock.locked()):
                    lock.acquire()
                    m.vibrate(1)
                    lock.release()
                print("switch")
            last[9] = True
            for i in range(9):
                last[i] = last[i+1]
        
        else:
            if (last==[True, True, True, True, True, True, True, True, True, True]):
                print("switch back")
                #hand = False
                if (not lock.locked()):
                    lock.acquire()
                    m.vibrate(1)
                    lock.release()
            #print("." + str(last))
            last[9] = False
            for i in range(9):
                last[i] = last[i+1]

        ## print framerate of received data
        times.append(time.time())
        if len(times) > 20:
            #print((len(times) - 1) / (times[-1] - times[0]))
            times.pop(0)
        
        #lock.release()

    m.add_emg_handler(proc_emg)
    m.connect()

    m.add_arm_handler(lambda arm, xdir: print('arm', arm, 'xdir', xdir))
    m.add_pose_handler(lambda p: print('pose', p))

    try:
        ###
        # Initialize pygame and SDL to use the PiTFT display and touchscreen.
        #os.putenv('SDL_VIDEODRIVER', 'fbcon')
        #os.putenv('SDL_FBDEV'      , '/dev/fb1')
        #os.putenv('SDL_MOUSEDRV'   , 'TSLIB')
        #os.putenv('SDL_MOUSEDEV'   , '/dev/input/touchscreen')
        pygame.display.init()
        pygame.font.init()
        #pygame.mouse.set_visible(False)
        # Get size of screen and create main rendering surface.
        size = (pygame.display.Info().current_w, pygame.display.Info().current_h)
        print ("w: " + str(size[0]))
        print ("h: " + str(size[1]))


        screen = pygame.display.set_mode(size, pygame.FULLSCREEN)
        # Display splash screen.
        splash = pygame.image.load('freqshow_splash.png')
        screen.fill(MAIN_BG)
        screen.blit(splash, ui.align(splash.get_rect(), (0, 0, size[0], size[1])))
        pygame.display.update()
        splash_start = time.time()
        # Create model and controller.
        
        fscontroller = controller.FreqShowController(fsmodel)
        #time.sleep(2.0)
        # Main loop to process events and render current view.
        lastclick = 0
            ###
        lock2 = Lock()
       # if (not lock2.locked()):
                #lock2.acquire()
        #print ("unlocked running")
        #m.start()
       
        #print ("joined")
        #lock2.release()
        while True:
            #thread.start_new_thread(m.run,(1,))
           
            m.act()
            #m.join()

            # Process any events (only mouse events for now).
            for event in pygame.event.get():
                if event.type is pygame.MOUSEBUTTONDOWN \
                    and (time.time() - lastclick) >= CLICK_DEBOUNCE:
                    lastclick = time.time()
                    fscontroller.current().click(pygame.mouse.get_pos())
            # Update and render the current view.
            
            fscontroller.multiview().render(screen)
            #for x in fscontroller.both_views():
            #   x.render(screen)

            #views.WaterfallSpectrogram(model, fscontroller).render(screen)
            #views.InstantSpectrogram(model, fscontroller).render(screen)
            pygame.display.update()
        m.end()

        #freqshow.run()
       
    except KeyboardInterrupt:
        pass
    finally:
        m.disconnect()
        print("DISCONNECTED")

