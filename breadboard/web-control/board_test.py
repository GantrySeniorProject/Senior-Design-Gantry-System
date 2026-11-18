# board_test.py - checks the serial protocol web-control/index.html uses, against the real board.
# The motors WILL move. Close the ESP-IDF monitor and the web page first (one program per port).
# Limits are pressed with the simulated px/py/pz commands, so no keys need pressing.
# Run with the ESP-IDF Python (has pyserial):
#   C:\Espressif\tools\python\v6.0\venv\Scripts\python.exe board_test.py [COMx]
import serial, serial.tools.list_ports, sys, time
ESP_VIDS = {0x1A86, 0x10C4, 0x0403, 0x303A}          # same USB chips the page looks for
AXES = "XYZ"
TRIPLES = ("pos", "steps", "span", "homed", "min", "max")
COORD_TOL = lambda span: 4096 // max(span, 1) + 1    # a coordinate lands on the nearest step

def find_port():
    if len(sys.argv) > 1: return sys.argv[1]
    ports = [p.device for p in serial.tools.list_ports.comports() if p.vid in ESP_VIDS]
    if not ports: sys.exit("No ESP32 USB-serial port found. Plug the board in or pass the port, e.g. COM8.")
    return ports[0]

def parse(line):
    """'@S mode=.. axis=.. phase=.. pos=x,y,z ... fault=..' -> dict; triples become int lists."""
    i = line.find("@S ")
    if i < 0: return None
    body = line[i+3:]; f = body.find("fault=")
    if f < 0: return None
    d = dict(t.split("=", 1) for t in body[:f].split() if "=" in t)
    try:
        for k in TRIPLES:
            d[k] = [int(v) for v in d[k].split(",")]
            if len(d[k]) != 3: return None
    except (KeyError, ValueError):
        return None
    d["fault"] = body[f+6:].strip()
    return d

if __name__ == "__main__" and "--selftest" in sys.argv:   # parser check, no board needed
    d = parse("g 1 2@S mode=FAULT axis=- phase=MAX pos=1,-1,4096 steps=3,0,9 span=9,0,9 homed=0,0,1 "
              "estop=0 min=0,1,0 max=0,0,0 keep=1 zon=0 path=2/5 fault=HOME TIMEOUT Z")
    assert d and d["pos"] == [1, -1, 4096] and d["homed"] == [0, 0, 1] and d["fault"] == "HOME TIMEOUT Z"
    assert d["keep"] == "1" and d["zon"] == "0" and d["axis"] == "-" and d["path"] == "2/5" and d["min"][1] == 1
    assert parse("@S mode=IDLE pos=1,2 steps=0,0,0 span=0,0,0 homed=0,0,0 min=0,0,0 max=0,0,0 fault=NONE") is None
    assert parse("X coord=1") is None
    print("selftest OK"); sys.exit(0)

s = serial.Serial(); s.port, s.baudrate, s.timeout = find_port(), 115200, 0.05
print("Using", s.port)
s.dtr = False; s.rts = False; s.open()
buf, st, n_status, fails = "", {}, 0, 0

def pump(sec):
    global buf, st, n_status
    end = time.time() + sec
    while time.time() < end:
        buf += s.read(4096).decode(errors="replace")
        while "\n" in buf:
            line, buf = buf.split("\n", 1)
            d = parse(line)
            if d: st = d; n_status += 1
def send(c): s.write((c + "\r\n").encode())
def until(pred, sec):
    end = time.time() + sec
    while time.time() < end:
        pump(0.05)
        if st and pred(st): return True
    return False
def check(ok, name):
    global fails
    fails += not ok
    print(("PASS  " if ok else "FAIL  ") + name, "|",
          {k: st.get(k) for k in ("mode", "axis", "phase", "pos", "steps", "span", "homed", "fault")}, flush=True)

def home_axis(a):
    """Drives one axis through homing with simulated presses. The axis must be next in line."""
    i = AXES.index(a); p = "p" + a.lower()
    check(until(lambda d: d["mode"] == "HOME" and d["axis"] == a and d["phase"] == "MIN", 3), f"homing {a}: seeking MIN")
    s0 = st["steps"][i]; pump(0.8)
    check(st["steps"][i] < s0 - 100, f"{a} turns toward MIN (steps {s0} -> {st['steps'][i]})")
    send(p + "0"); check(until(lambda d: d["axis"] == a and d["phase"] == "MAX", 2), f"{p}0 -> {a} zero found, seeking MAX")
    check(0 <= st["steps"][i] < 300, f"{a} step count restarted near 0")
    pump(1.5); s1 = st["steps"][i]; check(s1 > 200, f"{a} turns toward MAX (steps {s1})")
    send(p + "1"); check(until(lambda d: d["homed"][i] == 1, 2), f"{p}1 -> {a} homed")
    check(st["span"][i] >= s1, f"{a} span {st['span'][i]} measured")

send("x"); send("r"); pump(0.3); send("q1")
n_status = 0; pump(2.0)
check(30 <= n_status <= 50, f"status stream ~20 Hz ({n_status} lines in 2 s)")
check(st.get("mode") == "IDLE" and st.get("homed") == [0, 0, 0] and st.get("pos") == [-1, -1, -1],
      "starts idle, nothing homed, pos -1")

send("e"); check(until(lambda d: d["mode"] == "FAULT" and d["fault"] == "ESTOP", 2), "E-STOP (e) latches ESTOP fault")
send("h"); pump(0.5); check(st["mode"] == "FAULT", "HOME refused while faulted")
send("r"); check(until(lambda d: d["mode"] == "IDLE" and d["fault"] == "NONE", 2), "RESET (r) clears it")

# HOME ALL: Z first (if the firmware has the Z axis on), then X, then Y
ZON = st.get("zon", "1") == "1"
print("Z axis is", "ON" if ZON else "OFF (USE_Z_AXIS 0): Z steps skipped", flush=True)
HOMED_ALL = [1, 1, 1] if ZON else [1, 1, 0]
send("h")
for a in ("ZXY" if ZON else "XY"): home_axis(a)
check(until(lambda d: d["mode"] == "IDLE" and d["homed"] == HOMED_ALL, 20), f"HOME ALL done, homed {HOMED_ALL}")
check(st["pos"][:2] == [0, 0] and st["steps"][:2] == [0, 0], f"homing ends at 0,0 (pos {st['pos'][:2]})")
span = st["span"]; z0 = st["steps"][2]

# Grid moves (X/Y straight line) and Z column
send("g 0 0"); check(until(lambda d: d["mode"] == "IDLE" and d["pos"][:2] == [0, 0], 20), "grid 0,0")
check(st["steps"][:2] == [0, 0] and st["steps"][2] == z0, "steps 0,0 and Z untouched")
send("g 2048 2048"); check(until(lambda d: d["mode"] == "IDLE" and d["steps"][0] > 0, 20), "grid 2048,2048")
check(all(abs(st["steps"][i] - span[i] / 2) <= 1 for i in (0, 1)), f"middle of both rails ({st['steps'][:2]} of {span[:2]})")
if ZON:
    send("z 0"); check(until(lambda d: d["mode"] == "IDLE" and d["pos"][2] == 0, 20), "Z column 0 -> attachment down")
    check(st["steps"][2] == 0, "Z at step 0")
    send("g 50 50"); pump(0.3); send("z 4096"); pump(0.3)
    check(until(lambda d: d["mode"] == "IDLE", 20), "z refused while a grid move runs (busy), then idle")
    send("z 4096"); check(until(lambda d: d["mode"] == "IDLE" and d["pos"][2] == 4096, 20), "attachment back up")
else:
    send("z 0"); send("h z"); pump(0.6)
    check(st["mode"] == "IDLE" and st["steps"][2] == z0 and st["homed"][2] == 0, "Z off: z and h z refused, Z never moves")

# Paths (what the page's GO sends): w x1 y1 x2 y2 ...
send("g 0 0"); until(lambda d: d["mode"] == "IDLE" and d["pos"][:2] == [0, 0], 20)
send("w 128 0 128 1500 3000 1500")
seen = set()
def track(d):
    if d.get("path") and d["path"] != "0/0": seen.add(d["path"])
    return d["mode"] == "IDLE" and d.get("path") == "0/0"
check(until(lambda d: (track(d) and False) or d["mode"] == "MOVE", 2), "path starts")
check(until(track, 30), "path finishes, queue empty (path=0/0)")
check({"1/3", "2/3", "3/3"} <= seen, f"status walks through the 3 moves ({sorted(seen)})")
check(abs(st["pos"][0] - 3000) <= COORD_TOL(span[0]) and abs(st["pos"][1] - 1500) <= COORD_TOL(span[1]),
      f"ends at the last waypoint (pos {st['pos'][:2]})")
send("w 4096 1500 4096 4096 0 4096"); check(until(lambda d: d["mode"] == "MOVE", 2), "second path starts")
pump(0.6); send("x"); check(until(lambda d: d["mode"] == "IDLE" and d.get("path") == "0/0", 3), "STOP drops the rest of the path")
c2 = st["pos"]; pump(1.0); check(st["pos"] == c2 and st["mode"] == "IDLE", "nothing resumes after STOP")
for bad in ("w 5000 0", "w 100 200 300", "w", "w 1 2 x"):
    send(bad); pump(0.4); check(st["mode"] == "IDLE" and st["pos"] == c2, f"bad path refused: {bad!r}")
long_path = "w " + " ".join(f"{(k // 2) * 64} {((k + 1) // 2) % 2 * 256}" for k in range(64))
send(long_path); check(until(lambda d: d["mode"] == "MOVE", 3), f"64-waypoint path accepted ({len(long_path)} chars on one line)")
check(until(lambda d: d["mode"] == "IDLE" and d.get("path") == "0/0", 90), "64-waypoint path finishes")
check(abs(st["pos"][0] - 31 * 64) <= COORD_TOL(span[0]) and abs(st["pos"][1] - 0) <= COORD_TOL(span[1]),
      f"long path ends at its last point (pos {st['pos'][:2]})")
send("w " + " ".join(["0 0"] * 65)); pump(0.5); check(st["mode"] == "IDLE", "65 waypoints refused")

# Keep homing after an E-stop (r k)
c0 = st["pos"]
send("e"); check(until(lambda d: d["mode"] == "FAULT" and d["keep"] == "1", 2), "E-STOP while idle+homed: keep offered")
check(st["pos"] == c0, f"coords still shown while faulted ({st['pos']})")
send("r k"); check(until(lambda d: d["mode"] == "IDLE" and d["homed"] == HOMED_ALL, 2), "RESET (keep home) keeps homing")
check(st["pos"] == c0, f"same position after keep ({st['pos']} vs {c0})")
send("g 4096 4096"); check(until(lambda d: d["mode"] == "MOVE", 2), "grid move starts")
pump(0.4); send("e"); check(until(lambda d: d["mode"] == "FAULT" and d["keep"] == "1", 2), "E-STOP mid-move: keep offered")
c1 = st["pos"]; pump(0.5); check(st["pos"] == c1, f"stopped at {c1}")
send("r k"); check(until(lambda d: d["mode"] == "IDLE" and d["homed"] == HOMED_ALL and d["pos"] == c1, 2), "keep after mid-move E-stop")
send("g 0 0"); check(until(lambda d: d["mode"] == "IDLE" and d["steps"][:2] == [0, 0], 20), "moves work after keep")

# Limit hit mid-move: LIMIT X, no keep
send("g 4096 4096"); check(until(lambda d: d["mode"] == "MOVE", 2), "move toward MAX")
pump(0.3); send("px1"); check(until(lambda d: d["mode"] == "FAULT" and d["fault"] == "LIMIT X", 2), "X MAX hit mid-move -> LIMIT X")
check(st["keep"] == "0" and st["homed"] == [0, 0, 0], "LIMIT: every axis unhomed, keep not offered")
send("r k"); pump(0.5); check(st["mode"] == "FAULT", "r k refused after LIMIT")
send("r"); check(until(lambda d: d["mode"] == "IDLE" and d["fault"] == "NONE", 2), "plain RESET")
send("g 100 100"); pump(0.5); check(st["mode"] == "IDLE", "grid moves refused until homed again")

# Single-axis homing + simulated skip (on Z, or on Y while Z is off)
SA = "Z" if ZON else "Y"; si = "XYZ".index(SA)
send("h " + SA.lower()); home_axis(SA)
want = [0, 0, 0]; want[si] = 1
check(until(lambda d: d["mode"] == "IDLE", 20) and st["homed"] == want, f"h {SA.lower()} homes only {SA}")
if SA != "Z": check(st["pos"][si] == 0, f"h {SA.lower()} ends at 0")
if not ZON:                               # a g move needs X homed too
    send("h x"); home_axis("X"); check(until(lambda d: d["mode"] == "IDLE", 20), "X homed as well")
send(f"k {SA.lower()} 8"); pump(0.2); send("z 0" if ZON else "g 0 0")
check(until(lambda d: d["mode"] == "FAULT" and d["fault"] == "SKIP " + SA, 15), f"skipped steps -> SKIP {SA}")
check(st["keep"] == "0", "SKIP: keep not offered")
send("r"); check(until(lambda d: d["mode"] == "IDLE", 2), "reset")
pump(0.5); check(st["mode"] == "IDLE", "no instant re-fault after reset")

# E-stop during the first homing: nothing to keep
send("h"); check(until(lambda d: d["mode"] == "HOME", 2), "home again")
pump(0.3); send("e"); check(until(lambda d: d["mode"] == "FAULT", 2), "E-STOP while homing")
check(st["keep"] == "0", "E-stop during first homing: keep not offered")
send("r"); check(until(lambda d: d["mode"] == "IDLE", 2), "reset")

send("q0"); pump(0.3); n_status = 0; pump(1.0); check(n_status == 0, "q0 stops the status stream")
s.close(); print("SUMMARY:", "ALL PASS" if not fails else f"{fails} FAILED"); sys.exit(1 if fails else 0)
