#!/usr/bin/env python
'''
tool to manupulate ArduPilot firmware files, changing default parameters
'''

import os, sys, struct

import argparse

class embedded_defaults(object):
    def __init__(self, filename):
        self.filename = filename
        f = open(args.input_file,'r')
        self.firmware = f.read()
        f.close()
        self.offset = 0

    def find(self):
        '''find defaults in firmware'''
        magic_str = "PARMDEF"
        param_magic = [ 0x55, 0x37, 0xf4, 0xa0, 0x38, 0x5d, 0x48, 0x5b ]
        while True:
            i = self.firmware[self.offset:].find(magic_str)
            if i == -1:
                return None
            matched = True
            for j in range(len(param_magic)):
                if ord(self.firmware[self.offset+i+j+8]) != param_magic[j]:
                    matched = False
                    break
            if not matched:
                self.offset += i+8
                continue
            self.offset += i
            self.max_len, self.length = struct.unpack("<HH", self.firmware[self.offset+16:self.offset+20])
            return True
    
    def contents(self):
        '''return current contents'''
        return self.firmware[self.offset+20:self.offset+20+self.length]

    def set_file(self, filename):
        '''set defaults to contents of a file'''
        f = open(filename, 'r')
        contents = f.read()
        f.close()
        length = len(contents)
        if length > self.max_len:
            print("Error: Length %u larger than maximum %u" % (length, self.max_len))
            sys.exit(1)
        new_fw = self.firmware[:self.offset+18]
        new_fw += struct.pack("<H", length)
        new_fw += contents
        new_fw += self.firmware[self.offset+20+length:]
        self.firmware = new_fw

    def save(self):
        '''save new firmware'''
        f = open(self.filename, 'w')
        f.write(self.firmware)
        f.close()

def defaults_contents(firmware, ofs, length):
    '''return current defaults contents'''
    return firmware

parser = argparse.ArgumentParser()

parser.add_argument('input_file')
parser.add_argument('--set-file', type=str, default=None)
parser.add_argument('--show', action='store_true', default=False)

args = parser.parse_args()

defaults = embedded_defaults(args.input_file)

print("Loaded firmware of length %u" % len(defaults.firmware))

if not defaults.find():
    print("Param defaults not found in firmware")
    sys.exit(1)
    
print("Found param defaults max_length=%u length=%u" % (defaults.max_len, defaults.length))

if args.show:
    print(defaults.contents())

if args.set_file:
    defaults.set_file(args.set_file)
    defaults.save()
