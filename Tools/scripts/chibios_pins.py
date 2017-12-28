#!/usr/bin/env python
'''
setup board.h for chibios
'''

import optparse, sys, fnmatch

parser = optparse.OptionParser("chibios_pins.py")

opts, args = parser.parse_args()

if len(args) == 0:
        print("Usage: chibios_pins.py HWDEF")

# output variables for each pin
vtypes = ['MODER', 'OTYPER', 'OSPEEDR', 'PUPDR', 'ODR', 'AFRL', 'AFRH']
        
# number of pins in each port
pincount = { 'A': 16, 'B' : 16, 'C' : 16, 'D' : 16, 'E' : 16, 'F' : 16, 'G' : 16, 'H' : 2, 'I' : 0, 'J' : 0, 'K' : 0 }

ports = pincount.keys()

portmap = {}

# dictionary of all config lines, indexed by first word
lines = {}

class generic_pin(object):
        def __init__(self, port, pin, label, type, extra):
                self.port = port
                self.pin = pin
                self.label = label
                self.type = type
                self.extra = extra

        def get_AF(self):
                '''get alternate function number or None'''
                if len(self.extra) == 0 or not self.extra[0].startswith("AF"):
                        return None
                return int(self.extra[0][2:])

        def has_extra(self, v):
                return v in self.extra

        def get_MODER(self):
                '''return one of ALTERNATE, OUTPUT, ANALOG, INPUT'''
                if self.get_AF() is not None:
                        v = "ALTERNATE"
                elif self.type == 'OUTPUT':
                        v = "OUTPUT"
                elif self.type == 'ANALOG':
                        v = "ANALOG"
                elif self.has_extra("CS"):
                        v = "OUTPUT"
                else:
                        v = "INPUT"
                return "PIN_MODE_%s(%uU)" % (v, self.pin)

        def get_OTYPER(self):
                '''return one of PUSHPULL, OPENDRAIN'''
                return "PIN_OTYPE_PUSHPULL(%uU)" % self.pin

        def get_OSPEEDR(self):
                '''return one of SPEED_VERYLOW, SPEED_LOW, SPEED_MEDIUM, SPEED_HIGH'''
                values = ['SPEED_VERYLOW', 'SPEED_LOW', 'SPEED_MEDIUM', 'SPEED_HIGH']
                v = 'SPEED_HIGH'
                if self.has_extra("CS"):
                        v = "SPEED_MEDIUM"
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_O%s(%uU)" % (v, self.pin)

        def get_PUPDR(self):
                '''return one of FLOATING, PULLUP, PULLDOWN'''
                values = ['FLOATING', 'PULLUP', 'PULLDOWN']
                v = 'FLOATING'
                if self.has_extra("CS"):
                        v = "PULLUP"
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_PUPDR_%s(%uU)" % (v, self.pin)

        def get_ODR(self):
                '''return one of LOW, HIGH'''
                values = ['LOW', 'HIGH']
                v = 'HIGH'
                for e in self.extra:
                        if e in values:
                                v = e
                return "PIN_ODR_%s(%uU)" % (v, self.pin)

        def get_AFIO(self):
                '''return AFIO'''
                af = self.get_AF()
                if af is None:
                        af = 0
                return "PIN_AFIO_AF(%uU, %uU)" % (self.pin, af)

        def get_AFRL(self):
                '''return AFIO low 8'''
                if self.pin >= 8:
                        return None
                return self.get_AFIO()

        def get_AFRH(self):
                '''return AFIO high 8'''
                if self.pin < 8:
                        return None
                return self.get_AFIO()

        def __str__(self):
                return "P%s%u %s %s" % (self.port, self.pin, self.label, self.type)

# setup default as input pins
for port in ports:
        portmap[port] = []
        for pin in range(pincount[port]):
                portmap[port].append(generic_pin(port, pin, None, 'INPUT', []))

def process_line(line):
        '''process one line of pin definition file'''
        a = line.split()
        lines[a[0]] = a[1:]
        if a[0].startswith('P') and a[0][1] in ports:
                # it is a port/pin definition
                port = a[0][1]
                pin = int(a[0][2:])
                label = a[1]
                type = a[2]
                extra = a[3:]
                portmap[port][pin] = generic_pin(port, pin, label, type, extra)
        

f = open(args[0],"r")
for line in f.readlines():
        line = line.strip()
        if len(line) == 0 or line[0] == '#':
                continue
        process_line(line)

print('''
/*
 pin default setup - generated file, do not edit
*/

/*
 * I/O ports initial setup, this configuration is established soon after reset
 * in the initialization code.
 * Please refer to the STM32 Reference Manual for details.
 */
#define PIN_MODE_INPUT(n)           (0U << ((n) * 2U))
#define PIN_MODE_OUTPUT(n)          (1U << ((n) * 2U))
#define PIN_MODE_ALTERNATE(n)       (2U << ((n) * 2U))
#define PIN_MODE_ANALOG(n)          (3U << ((n) * 2U))
#define PIN_ODR_LOW(n)              (0U << (n))
#define PIN_ODR_HIGH(n)             (1U << (n))
#define PIN_OTYPE_PUSHPULL(n)       (0U << (n))
#define PIN_OTYPE_OPENDRAIN(n)      (1U << (n))
#define PIN_OSPEED_VERYLOW(n)       (0U << ((n) * 2U))
#define PIN_OSPEED_LOW(n)           (1U << ((n) * 2U))
#define PIN_OSPEED_MEDIUM(n)        (2U << ((n) * 2U))
#define PIN_OSPEED_HIGH(n)          (3U << ((n) * 2U))
#define PIN_PUPDR_FLOATING(n)       (0U << ((n) * 2U))
#define PIN_PUPDR_PULLUP(n)         (1U << ((n) * 2U))
#define PIN_PUPDR_PULLDOWN(n)       (2U << ((n) * 2U))
#define PIN_AFIO_AF(n, v)           ((v) << (((n) % 8U) * 4U))

''')

for port in sorted(ports):
        print("/* PORT%s:" % port)
        for pin in range(pincount[port]):
                p = portmap[port][pin]
                if p.label is not None:
                        print(" %s" % p)
        print("*/\n")

        if pincount[port] == 0:
                '''blank ports'''
                for vtype in vtypes:
                        print("#define VAL_GPIO%s_%-7s             0x0" % (port, vtype))
                print("\n\n")
                continue

        for vtype in vtypes:
                if getattr(portmap[port][0], "get_" + vtype, None) is None:
                        continue
                sys.stdout.write("#define VAL_GPIO%s_%-7s (" % (p.port, vtype))
                first = True
                for pin in range(pincount[port]):
                        p = portmap[port][pin]
                        modefunc = getattr(p, "get_" + vtype)
                        v = modefunc()
                        if v is None:
                                continue
                        if not first:
                                sys.stdout.write(" | \\\n                           ")
                        sys.stdout.write(v)
                        first = False
                if first:
                        sys.stdout.write("0")
                print(")\n")
