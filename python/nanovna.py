#!/usr/bin/env python3
import serial
import numpy as np
import pylab as pl
import scipy.signal as signal
import struct
from serial.tools import list_ports

VID = 0x0483 #1155
PID = 0x5740 #22336

# Get nanovna device automatically
def getport() -> str:
    device_list = list_ports.comports()
    for device in device_list:
        if device.vid == VID and device.pid == PID:
            return device.device
    raise OSError("device not found")

REF_LEVEL = (1<<9)

# b, a = signal.ellip(4, 0.2, 100, (4700.0/24000, 5100.0/24000), 'bandpass')
# def bandpassfilter_5khz(ref, samp):
#     zi = signal.lfiltic(b, a, np.ones([0]))
#     samp1,zi = signal.lfilter(b, a, samp, zi = zi)
#     samp1,x = signal.lfilter(b, a, samp, zi = zi)
#     zi = signal.lfiltic(b, a, np.ones([0]))
#     ref1,zi = signal.lfilter(b, a, ref, zi = zi)
#     ref1,x = signal.lfilter(b, a, ref, zi = zi)
#     return ref1,samp1

class NanoVNA:
    def __init__(self, dev):
        self.dev = dev
        self.serial = None
        self.filter = None # bandpassfilter_5khz
        self._frequencies = None
        self.points = 101
        
    @property
    def frequencies(self):
        return self._frequencies

    def set_frequencies(self, start = 1e6, stop = 900e6, points = None):
        if points:
            self.points = points
        self._frequencies = np.linspace(start, stop, self.points)

    def open(self):
        if self.serial is None:
            self.serial = serial.Serial(self.dev)

    def close(self):
        if self.serial:
            self.serial.close()
        self.serial = None

    def send_command(self, cmd):
        self.open()
        self.serial.write(cmd.encode())
        self.serial.readline() # discard empty line

    def set_sweep(self, start, stop):
        if start is not None:
            self.send_command("sweep start %d\r" % start)
        if stop is not None:
            self.send_command("sweep stop %d\r" % stop)

    def set_frequency(self, freq):
        if freq is not None:
            self.send_command("freq %d\r" % freq)

    def set_port(self, port):
        if port is not None:
            self.send_command("port %d\r" % port)

    def set_gain(self, gain):
        if gain is not None:
            self.send_command("gain %d %d\r" % (gain, gain))

    def set_offset(self, offset):
        if offset is not None:
            self.send_command("offset %d\r" % offset)

    def set_strength(self, strength):
        if strength is not None:
            self.send_command("power %d\r" % strength)

    def set_filter(self, filter):
        self.filter = filter

    def fetch_data(self):
        result = ''
        line = ''
        while True:
            c = self.serial.read().decode('utf-8')
            if c == chr(13):
                next # ignore CR
            line += c
            if c == chr(10):
                result += line
                line = ''
                next
            if line.endswith('ch>'):
                # stop on prompt
                break
        return result

    def fetch_buffer(self, freq = None, buffer = 0):
        self.send_command("dump %d\r" % buffer)
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                x.extend([int(d, 16) for d in line.strip().split(' ')])
        return np.array(x, dtype=np.int16)

    def fetch_rawwave(self, freq = None):
        if freq:
            self.set_frequency(freq)
            time.sleep(0.05)
        self.send_command("dump 0\r")
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                x.extend([int(d, 16) for d in line.strip().split(' ')])
        return np.array(x[0::2], dtype=np.int16), np.array(x[1::2], dtype=np.int16)

    def fetch_array(self, sel):
        self.send_command("data %d\r" % sel)
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                x.extend([float(d) for d in line.strip().split(' ')])
        return np.array(x[0::2]) + np.array(x[1::2]) * 1j

    def fetch_gamma(self, freq = None):
        if freq:
            self.set_frequency(freq)
        self.send_command("gamma\r")
        data = self.serial.readline()
        d = data.strip().split(' ')
        return (int(d[0])+int(d[1])*1.j)/REF_LEVEL

    def reflect_coeff_from_rawwave(self, freq = None):
        ref, samp = self.fetch_rawwave(freq)
        if self.filter:
            ref, samp = self.filter(ref, samp)
        refh = signal.hilbert(ref)
        #x = np.correlate(refh, samp) / np.correlate(refh, refh)
        #return x[0]
        #return np.sum(refh*samp / np.abs(refh) / REF_LEVEL)
        return np.average(refh*samp / np.abs(refh) / REF_LEVEL)

    reflect_coeff = reflect_coeff_from_rawwave
    gamma = reflect_coeff_from_rawwave
    #gamma = fetch_gamma
    coefficient = reflect_coeff

    def resume(self):
        self.send_command("resume\r")
    
    def pause(self):
        self.send_command("pause\r")
    
    def scan_gamma0(self, port = None):
        self.set_port(port)
        return np.vectorize(self.gamma)(self.frequencies)

    def scan_gamma(self, port = None):
        self.set_port(port)
        return np.vectorize(self.fetch_gamma)(self.frequencies)

    def data(self, array = 0):
        self.send_command("data %d\r" % array)
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                d = line.strip().split(' ')
                x.append(float(d[0])+float(d[1])*1.j)
        return np.array(x)

    def fetch_frequencies(self):
        self.send_command("frequencies\r")
        data = self.fetch_data()
        x = []
        for line in data.split('\n'):
            if line:
                x.append(float(line))
        self._frequencies = np.array(x)

    def send_scan(self, start = 1e6, stop = 900e6, points = None):
        if points:
            self.send_command("scan %d %d %d\r"%(start, stop, points))
        else:
            self.send_command("scan %d %d\r"%(start, stop))

    def scan(self):
        segment_length = 101
        array0 = []
        array1 = []
        freqs = self._frequencies
        while len(freqs) > 0:
            seg_start = freqs[0]
            seg_stop = freqs[segment_length-1] if len(freqs) >= segment_length else freqs[-1]
            length = segment_length if len(freqs) >= segment_length else len(freqs)
            #print((seg_start, seg_stop, length))
            self.send_scan(seg_start, seg_stop, length)
            array0.extend(self.data(0))
            array1.extend(self.data(1))
            freqs = freqs[segment_length:]
        self.resume()
        return (array0, array1)
    
    def capture(self):
        from PIL import Image
        self.send_command("capture\r")
        b = self.serial.read(320 * 240 * 2)
        x = struct.unpack(">76800H", b)
        # convert pixel format from 565(RGB) to 8888(RGBA)
        arr = np.array(x, dtype=np.uint32)
        arr = 0xFF000000 + ((arr & 0xF800) >> 8) + ((arr & 0x07E0) << 5) + ((arr & 0x001F) << 19)
        return Image.frombuffer('RGBA', (320, 240), arr, 'raw', 'RGBA', 0, 1)

    def logmag(self, x):
        pl.grid(True)
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, 20*np.log10(np.abs(x)))

    def linmag(self, x):
        pl.grid(True)
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, np.abs(x))

    def phase(self, x, unwrap=False):
        pl.grid(True)
        a = np.angle(x)
        if unwrap:
            a = np.unwrap(a)
        else:
            pl.ylim((-180,180))
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, np.rad2deg(a))

    def delay(self, x):
        pl.grid(True)
        delay = -np.unwrap(np.angle(x))/ (2*np.pi*np.array(self.frequencies))
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, delay)

    def groupdelay(self, x):
        pl.grid(True)
        gd = np.convolve(np.unwrap(np.angle(x)), [1,-1], mode='same')
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, gd)

    def vswr(self, x):
        pl.grid(True)
        vswr = (1+np.abs(x))/(1-np.abs(x))
        pl.xlim(self.frequencies[0], self.frequencies[-1])
        pl.plot(self.frequencies, vswr)

    def polar(self, x):
        ax = pl.subplot(111, projection='polar')
        ax.grid(True)
        ax.set_ylim((0,1))
        ax.plot(np.angle(x), np.abs(x))

    def tdr(self, x):
        pl.grid(True)
        window = np.blackman(len(x))
        NFFT = 256
        td = np.abs(np.fft.ifft(window * x, NFFT))
        time = 1 / (self.frequencies[1] - self.frequencies[0])
        t_axis = np.linspace(0, time, NFFT)
        pl.plot(t_axis, td)
        pl.xlim(0, time)
        pl.xlabel("time (s)")
        pl.ylabel("magnitude")

    def smithd3(self, x):
        import mpld3
        import twoport as tp
        fig, ax = pl.subplots()
        sc = tp.SmithChart(show_cursor=True, labels=True, ax=ax)
        sc.plot_s_param(a)
        mpld3.display(fig)

    def skrf_network(self, x):
        import skrf as sk
        n = sk.Network()
        n.frequency = sk.Frequency.from_f(self.frequencies / 1e6, unit='mhz')
        n.s = x
        return n

    def smith(self, x):
        n = self.skrf_network(x)
        n.plot_s_smith()
        return n

def plot_sample0(samp):
    N = min(len(samp), 256)
    fs = 48000
    pl.subplot(211)
    pl.grid()
    pl.plot(samp)
    pl.subplot(212)
    pl.grid()
    #pl.ylim((-50, 50))
    pl.psd(samp, N, window = pl.blackman(N), Fs=fs)

def plot_sample(ref, samp):
    N = min(len(samp), 256)
    fs = 48000
    pl.subplot(211)
    pl.grid()
    pl.plot(ref)
    pl.plot(samp)
    pl.subplot(212)
    pl.grid()
    #pl.ylim((-50, 50))
    pl.psd(ref, N, window = pl.blackman(N), Fs=fs)
    pl.psd(samp, N, window = pl.blackman(N), Fs=fs)

if __name__ == '__main__':
    from optparse import OptionParser
    parser = OptionParser(usage="%prog: [options]")
    parser.add_option("-f", "--file", dest="filename",
                      help="read from FILE", metavar="FILE")
    parser.add_option("-r", "--raw", dest="rawwave",
                      type="int", default=None,
                      help="plot raw waveform", metavar="RAWWAVE")
    parser.add_option("-p", "--plot", dest="plot",
                      action="store_true", default=False,
                      help="plot rectanglar", metavar="PLOT")
    parser.add_option("-s", "--smith", dest="smith",
                      action="store_true", default=False,
                      help="plot smith chart", metavar="SMITH")
    parser.add_option("-L", "--polar", dest="polar",
                      action="store_true", default=False,
                      help="plot polar chart", metavar="POLAR")
    parser.add_option("-D", "--delay", dest="delay",
                      action="store_true", default=False,
                      help="plot delay", metavar="DELAY")
    parser.add_option("-G", "--groupdelay", dest="groupdelay",
                      action="store_true", default=False,
                      help="plot groupdelay", metavar="GROUPDELAY")
    parser.add_option("-W", "--vswr", dest="vswr",
                      action="store_true", default=False,
                      help="plot VSWR", metavar="VSWR")
    parser.add_option("-H", "--phase", dest="phase",
                      action="store_true", default=False,
                      help="plot phase", metavar="PHASE")
    parser.add_option("-U", "--unwrapphase", dest="unwrapphase",
                      action="store_true", default=False,
                      help="plot unwrapped phase", metavar="UNWRAPPHASE")
    parser.add_option("-T", "--timedomain", dest="tdr",
                      action="store_true", default=False,
                      help="plot TDR", metavar="TDR")
    parser.add_option("-c", "--scan", dest="scan",
                      action="store_true", default=False,
                      help="scan by script", metavar="SCAN")
    parser.add_option("-S", "--start", dest="start",
                      type="float", default=1e6,
                      help="start frequency", metavar="START")
    parser.add_option("-E", "--stop", dest="stop",
                      type="float", default=900e6,
                      help="stop frequency", metavar="STOP")
    parser.add_option("-N", "--points", dest="points",
                      type="int", default=101,
                      help="scan points", metavar="POINTS")
    parser.add_option("-P", "--port", type="int", dest="port",
                      help="port", metavar="PORT")
    parser.add_option("-d", "--dev", dest="device",
                      help="device node", metavar="DEV")
    parser.add_option("-F", "--freqeucy", type="int", dest="freq",
                      help="frequency", metavar="FREQ")
    parser.add_option("-g", "--gain", type="int", dest="gain",
                      help="gain (0-95)", metavar="GAIN")
    parser.add_option("-O", "--offset", type="int", dest="offset",
                      help="offset frequency", metavar="OFFSET")
    parser.add_option("--strength", type="int", dest="strength",
                      help="drive strength(0-3)", metavar="STRENGTH")
    parser.add_option("-v", "--verbose",
                      action="store_true", dest="verbose", default=False,
                      help="verbose output")
    parser.add_option("-l", "--filter",
                      action="store_true", dest="filter", default=False,
                      help="apply IF filter on raw wave plot")
    parser.add_option("-C", "--capture", dest="capture",
                      help="capture current display to FILE", metavar="FILE")
    (opt, args) = parser.parse_args()

    nv = NanoVNA(opt.device or getport())

    if opt.capture:
        print("capturing...")
        img = nv.capture()
        img.save(opt.capture)
        exit(0)

    nv.set_frequency(opt.freq)
    nv.set_port(opt.port)
    nv.set_gain(opt.gain)
    nv.set_offset(opt.offset)
    nv.set_strength(opt.strength)
    if opt.rawwave is not None:
        samp = nv.fetch_buffer(buffer = opt.rawwave)
        print(len(samp))
        if opt.rawwave == 1 or opt.rawwave == 2:
            plot_sample0(samp)
            print(np.average(samp))
        else:
            plot_sample(samp[0::2], samp[1::2])
            print(np.average(samp[0::2]))
            print(np.average(samp[1::2]))
            print(np.average(samp[0::2] * samp[1::2]))
        pl.show()
        exit(0)
    if opt.start or opt.stop or opt.points:
        nv.set_frequencies(opt.start, opt.stop, opt.points)
    plot = opt.phase or opt.plot or opt.vswr or opt.delay or opt.groupdelay or opt.smith or opt.unwrapphase or opt.polar or opt.tdr
    if plot:
        p = int(opt.port) if opt.port else 0
        if opt.scan or opt.points > 101:
            s = nv.scan()
            s = s[p]
        else:
            if opt.start or opt.stop:
                nv.set_sweep(opt.start, opt.stop)
            nv.fetch_frequencies()
            s = nv.data(p)
            nv.fetch_frequencies()
    if opt.smith:
        nv.smith(s)
    if opt.polar:
        nv.polar(s)
    if opt.plot:
        nv.logmag(s)
    if opt.phase:
        nv.phase(s)
    if opt.unwrapphase:
        nv.phase(s, unwrap=True)
    if opt.delay:
        nv.delay(s)
    if opt.groupdelay:
        nv.groupdelay(s)
    if opt.vswr:
        nv.vswr(s)
    if opt.tdr:
        nv.tdr(s)
    if plot:
        pl.show()
