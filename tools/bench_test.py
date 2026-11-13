import serial, time, sys, re
PORT='/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0'
s=serial.Serial(PORT,115200,timeout=0.1)
buf=b''
def pump(sec):
    global buf
    t=time.time()
    while time.time()-t<sec: buf+=s.read(512)
def lines():
    return [l.strip() for l in buf.decode('ascii','replace').split('\n')[:-1] if l.strip()]
def last():
    for l in reversed(lines()):
        if l.startswith('mode='):
            return {k:v for k,v in re.findall(r'(\w+)=(\S+)',l)}, l
    return None,None
fails=0
def step(name, cmd, wait, **exp):
    global fails
    m=re.match(r't([\d.]+)\n',cmd or '')
    if m:
        for _ in range(4):
            s.write(m.group(0).encode()); pump(0.8)
            if abs(float(last()[0]['tH'])-float(m.group(1)))<0.06: break
            print('     (temp command garbled on UART, resending)')
        cmd=cmd[m.end():]
    if cmd: s.write(cmd.encode())
    pump(wait)
    d,l=last()
    ok=True; why=[]
    for k,v in exp.items():
        got=d.get(k)
        if callable(v):
            if not v(got): ok=False; why.append(f'{k}={got}')
        elif got!=str(v): ok=False; why.append(f'{k}={got} (want {v})')
    if not ok: fails+=1
    print(('PASS ' if ok else 'FAIL ')+name+('  <- '+', '.join(why) if why else ''))
    print('     '+l)
    return d
t0=time.time(); pump(6)
ls=lines()
print('boot lines:',[l for l in ls if not l.startswith('mode=')])
print('--- 5 s on real (floating) inputs:')
for l in [l for l in ls if l.startswith('mode=')][:8]: print('     '+l)
# sim mode
s.write(b's t25\n'); pump(1.2)
for _ in range(3):
    d,_l=last()
    if d['mode']=='0': break
    s.write(b'b'); pump(1.2)
OFF=dict(pwm=0,com2b=0,d3=0,d5=0)    # heater off, fan off (nothing heated yet)
HOFF=dict(pwm=0,com2b=0,d3=0,d5=1)   # heater off, fan in run-on
step('IDLE: heater+fan off, pullups A1/A3 high','',1.2,mode=0,a1=1,a3=1,**OFF)
step('IDLE: UP/DOWN/BACK ignored','udb',1.2,mode=0)
step('OK -> MENU','o',1.2,mode=1,sel=0,**OFF)
step('DOWN x6 clamps at last profile','d',1.1); [step('  down','d',1.1) for _ in range(4)]
step('  sel==4','d',1.0,mode=1,sel=4)
[step('  up','u',1.1) for _ in range(5)]
step('UP x6 clamps at 0','u',1.0,mode=1,sel=0)
step('BACK -> IDLE','b',1.2,mode=0)
step('OK -> MENU','o',1.0,mode=1)
step('select PETG','d',1.0,sel=1)
step('OK -> RUN, cold: full power, fan on','o',2.5,mode=2,pwm=255,d3=1,d5=1,left=lambda v:21590<int(v)<=21600)
step('near setpoint (64C): partial PWM on Timer2','t64\n',3.5,mode=2,pwm=lambda v:0<int(v)<255,com2b=1,d5=1)
d,_=last(); print('     ocr2b==pwm:', d['ocr2b']==d['pwm']); fails+= d['ocr2b']!=d['pwm']
step('above setpoint (70C): heater off, fan stays on','t70\n',3.5,mode=2,pwm=0,d3=0,com2b=0,d5=1)
step('back to 60C: heater on again','t60\n',3.5,mode=2,pwm=lambda v:int(v)>0,d5=1)
step('fast-forward 5 h: still RUN','fffff',1.5,mode=2,left=lambda v:0<int(v)<=3600)
step('fast-forward past end -> DONE, outputs off','f',2.5,mode=3,**HOFF)
step('DONE: OK -> MENU','o',1.2,mode=1)
step('MENU: OK -> RUN','o',2.5,mode=2,pwm=lambda v:int(v)>0)
step('RUN: BACK aborts -> IDLE, outputs off','b',1.5,mode=0,**HOFF)
s.write(b'o'); pump(0.6); step('start RUN again (OK, OK)','o',2.5,mode=2,pwm=lambda v:int(v)>0)
step('overtemp 95C -> FAULT, outputs off','t95\n',1.5,mode=4,**HOFF)
step('BACK while still hot -> re-faults','b',1.5,mode=4,**HOFF)
step('cooled to 25C, BACK -> IDLE','t25\nb',1.5,mode=0,**HOFF)
s.write(b'o'); pump(0.6); step('start RUN again (OK, OK)','o',2.5,mode=2,pwm=lambda v:int(v)>0)
step('thermistor NaN -> FAULT, outputs off','n',1.5,mode=4,**HOFF)
step('NaN persists: BACK cannot leave FAULT','b',1.5,mode=4,**HOFF)
step('sensor back, BACK -> IDLE','t25\nb',1.5,mode=0,**HOFF)
step('boundary 90.0C is not a fault','t90\n',1.2,mode=0)
step('90.5C in IDLE -> FAULT','t90.5\n',1.2,mode=4)
d=step('recover','t25\nb',1.5,mode=0)
s.write(b'o'); pump(0.6)
step('heater watch: full power, no temp rise for window (20 s in debug) -> FAULT','o',25,mode=4,hf=1,**HOFF)
step('BACK clears heater fault -> IDLE','b',1.5,mode=0,hf=0,**HOFF)
step('fan run-on ends 3 min after heater off (25C)','',182,mode=0,**OFF)
step('heater still hot (45C) keeps fan on in IDLE','t45\n',1.2,mode=0,pwm=0,d3=0,d5=1)
step('38C: inside hysteresis band, fan still on','t38\n',1.2,mode=0,d5=1)
step('cooled (25C): fan off','t25\n',1.2,mode=0,**OFF)
ls=[l for l in lines() if l.startswith('mode=')]
used=max(int(re.search(r'stkUsed=(\d+)',l).group(1)) for l in ls)
lm=sorted(int(re.search(r'loopMax=(\d+)',l).group(1)) for l in ls)
print(f'max stack used: {used} B; loop period ms: median {lm[len(lm)//2]}, max {lm[-1]}')
print('resets during test (BOOT count):', sum(1 for l in lines() if l=='BOOT'))
n0=len(buf); s.write(b'x'); pump(8)
t=buf[n0:].decode('ascii','replace'); i=t.find('WDT d3=')
ok = i>=0 and t[i+7:i+9]=='01' and 'mode=' not in t[i:]
fails += (not ok)
print(('PASS ' if ok else 'FAIL ')+'watchdog: stalled loop -> heater pin low, fan pin high, firmware halted'); print('     '+repr(t[max(i,0):][:40]))
s.close()
print('FAILS:',fails)
