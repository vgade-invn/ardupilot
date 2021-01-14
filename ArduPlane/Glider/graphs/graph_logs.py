#!/usr/bin/env python

import sys, random, os, time, multiprocessing, subprocess
from pymavlink import mavutil
from MAVProxy.modules.lib import grapher
import common

from argparse import ArgumentParser
parser = ArgumentParser(description=__doc__)

parser.add_argument("--newer", action='store_true')
parser.add_argument("--parallel", default=6, type=int)
parser.add_argument("files", type=str, nargs='+', help="input files")
args = parser.parse_args()

files = list(dict.fromkeys(args.files))
nfiles = len(files)

graphed = set()

graphs = [
    ('TECS Speed', 'knots(TECS.sp)/CTUN.E2T<KEAS> knots(TECS.spdem)/CTUN.E2T<KEAS_Target> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Attitude Control', 'ATT.Roll ATT.Pitch ATT.DesRoll ATT.DesPitch feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Speeds', 'knots(TECS.sp)/CTUN.E2T<KEAS> knots(TECS.sp)<KTAS> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Surfaces Deg', 'AETR.Elev*12.5/4500+1.25<Elevator(deg)> AETR.Ail*15.5/4500<Aileron(deg)> AETR.Rudd*18.5/4500<Rudder(deg)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Surfaces PWM', 'RCOU.C3<Elevator(PWM)> RCOU.C4<Rudder(PWM)> RCOU.C2<AileronLeft(PWM)> RCOU.C5<AileronRight(PWM)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Angle of Attack', 'constrain(SLD.AoA,-15,15)<AOA(deg)> constrain(AOA.SSA,-15,15){abs(AOA.AOA)<10}<SSA(deg)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('G-Force', 'sqrt(IMU.AccX**2+IMU.AccY**2+IMU.AccZ**2)<AccelLength(m/s/s)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Distance To Home', 'distance_lat_lon(GPS.Lat,GPS.Lng,34.905429,-117.883702)/1852.0<DistToHome(nm)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Air Density', 'SLD.AD<AirDensity(kg/m^3)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Dynamic Pressure', 'SLD.q<DynamicPressure(Pa)> feet(GPS.Alt):2<AltAMSL(Ft)>'),
    ('Lift/Drag', 'SLD.LD<Lift/Drag> feet(GPS.Alt):2<AltAMSL(Ft)>'),
]

def graph_one(mlog, title, expression, filename):
    '''create one graph'''
    print("Graphing %s to %s" % (title, filename))
    mg = grapher.MavGraph()
    mg.add_mav(mlog)
    for e in expression.split():
        mg.add_field(e)
    mg.set_title(title)
    mg.process([],[],0)
    mg.set_grid(True)
    mg.show(1, output=filename)

def script_path():
    '''path of this script'''
    import inspect
    return os.path.abspath(inspect.getfile(inspect.currentframe()))

def process_one(fname):
    '''process one file'''
    html = fname + ".html"
    print("Graphing %s %u/%u" % (fname, len(graphed), len(files)))

    tmp = html+".tmp"+str(random.randint(0,100000))
    tmpg = tmp+".png"
    try:
        os.unlink(tmp)
    except Exception:
        pass
    try:
        os.unlink(tmpg)
    except Exception:
        pass

    bname = os.path.basename(fname)
    bname2 = bname[:8]
    dname = os.path.dirname(fname)
    f = open(tmp, "w")
    f.write("<html><head><title>Graphs of %s</title><body>\n" % bname)
    f.write("<h1>Graphs of log %s</h1>\n" % bname)
    f.write('Logfile: <a href="../all/%s" target="_blank">%s</a><br>\n' % (bname, bname))
    f.write("<pre>")
    proc = subprocess.Popen("mavflighttime.py %s | egrep '^Flig|^Total.dist'" % fname, shell=True, stdout=subprocess.PIPE)
    f.write(str(proc.stdout.read()))
    f.write("</pre><p>")

    map_log = "%s/%s.bin" % (dname, bname2)
    map_img = "%s-map.png" % bname

    os.system("mavflightview.py --imagefile=%s/%s %s" % (dname, map_img, map_log))
    f.write('<hr><p><img src="%s"><p>\n' % map_img)

    mlog = mavutil.mavlink_connection(fname)
    idx = 0
    for (title, expression) in graphs:
        idx += 1
        if os.path.exists(tmpg):
            os.unlink(tmpg)
        p = multiprocessing.Process(target=graph_one, args=(mlog, title, expression, tmpg))
        p.start()
        p.join()
        if os.path.exists(tmpg):
            os.rename(tmpg, "%s/%s-g%u.png" % (dname, bname, idx))
            png_name = "%s-g%u.png" % (bname, idx)
            f.write('<a href="%s"><img src="%s" width=1035 height=527></a><p>\n' % (png_name, png_name))
    f.write("</body>\n")
    f.close()
    if os.path.exists(tmp):
        try:
            os.unlink(html)
        except Exception:
            pass
        os.rename(tmp, html)

def is_done(fname):
    '''check if already done'''
    return False

files2 = []
for f in files:
    if os.path.getsize(f) == 0:
        continue
    if not is_done(f + ".html"):
        files2.append(f)
files = files2
nfiles = len(files2)

procs = []

while len(graphed) < nfiles:
    r = random.randint(0, nfiles-1)
    fname = files[r]
    if fname in graphed:
        continue
    graphed.add(fname)
    html = fname + ".html"
    if is_done(html):
        continue
    p = multiprocessing.Process(target=process_one, args=(fname,))
    p.start()
    procs.append(p)
    while len(procs) >= args.parallel:
        for p in procs:
            if p.exitcode is not None:
                p.join()
                procs.remove(p)
        time.sleep(0.1)
