#!/usr/bin/env python

'''
check that replay produced identical results
'''

import time

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)
parser.add_argument("--condition", default=None, help="condition for packets")
parser.add_argument("--ekf2-only", action='store_true', help="only check EKF2")
parser.add_argument("--ekf3-only", action='store_true', help="only check EKF3")
parser.add_argument("logs", metavar="LOG", nargs="+")

args = parser.parse_args()

from pymavlink import mavutil


def check_log(logfile):
    '''check replay log for matching output'''
    print("Processing log %s" % filename)

    mlog = mavutil.mavlink_connection(filename)

    ek2_list = ['NKF1','NKF2','NKF3','NKF4']
    ek3_list = ['XKF1','XKF2','XKF3','XKF4']
    
    if args.ekf2_only:
        mlist = ek2_list
    elif args.ekf3_only:
        mlist = ek3_list
    else:
        mlist = ek2_list + ek3_list

    base = {}
    for m in mlist:
        base[m] = {}

    while True:
        m = mlog.recv_match(type=mlist)
        if m is None:
            break
        if not hasattr(m,'C'):
            continue
        mtype = m.get_type()
        core = m.C
        if core < 100:
            base[mtype][core] = m
            continue
        mb = base[mtype][core-100]
        for f in m._fieldnames:
            if f == 'C':
                continue
            v1 = getattr(m,f)
            v2 = getattr(mb,f)
            if v1 != v2:
                print("Mismatch in field %s.%s: %s %s" % (mtype, f, str(v1), str(v2)))
                print(mb)
                print(m)

for filename in args.logs:
    check_log(filename)
