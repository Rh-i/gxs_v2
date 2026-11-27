#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
上下位机串口协议测试工具（纯标准库，不需要 pyserial）

帧格式：AA 55 | TYPE | SEQ | LEN | PAYLOAD | SUM | 0D 0A
  - SUM = 从 TYPE 到 PAYLOAD 末字节的 8 位累加和
  - 多字节小端

用法：
  python3 tools/proto_test.py -p /dev/ttyACM0 ping
  python3 tools/proto_test.py -p /dev/ttyACM0 monitor            # 只收，打印所有帧
  python3 tools/proto_test.py -p /dev/ttyACM0 vel 0 50 0 --hz 20 --sec 2
  python3 tools/proto_test.py -p /dev/ttyACM0 move 100 0
  python3 tools/proto_test.py -p /dev/ttyACM0 servo 1 1350
  python3 tools/proto_test.py -p /dev/ttyACM0 estop
  python3 tools/proto_test.py -p /dev/ttyACM0 selftest           # 全流程自动跑一遍
"""

import argparse
import os
import select
import struct
import sys
import termios
import time

SOF1, SOF2 = 0xAA, 0x55
EOF1, EOF2 = 0x0D, 0x0A
MAX_PAYLOAD = 48

# 上位机 → 车
CMD_VEL = 0x01
CMD_VEL_TIME = 0x02
CMD_MOVE = 0x03
CMD_SERVO = 0x04
PING = 0x05
HOST_ACK = 0xAC
ESTOP = 0xFF

# 车 → 上位机
ACK = 0x00
FIELD_STATUS = 0x01
STATE = 0x02
EVENT = 0x06
MOVE_DONE = 0x08
PONG = 0x0F

# 上行/下行有同号不同义的 TYPE（0x01、0x02），分开查表
NAMES_UP = {
    CMD_VEL: "CMD_VEL", CMD_VEL_TIME: "CMD_VEL_TIME", CMD_MOVE: "CMD_MOVE",
    CMD_SERVO: "CMD_SERVO", PING: "PING", HOST_ACK: "HOST_ACK", ESTOP: "ESTOP",
}
NAMES_DOWN = {
    ACK: "ACK", FIELD_STATUS: "FIELD_STATUS", STATE: "STATE", EVENT: "EVENT",
    MOVE_DONE: "MOVE_DONE", PONG: "PONG",
}
NAMES = dict(NAMES_UP)
NAMES.update(NAMES_DOWN)  # 只给没冲突的地方用；打印请用 NAMES_UP / NAMES_DOWN
RESULT = {0: "OK", 1: "BAD_FRAME", 2: "UNSUPPORTED", 3: "REJECTED"}
SOURCE = {0: "RC", 1: "HOST", 2: "STOP"}

# 舵机保守范围（单位 0.1°）：100°~170°；中位 135°
SERVO_MIN, SERVO_MAX = 1000, 1700


# ------------------------------------------------------------------ 串口

class Serial:
    """只要能读写、不阻塞，就够用了"""

    def __init__(self, port, baud=921600):
        self.fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        attr = termios.tcgetattr(self.fd)
        attr[0] = 0                                            # iflag: 原始模式
        attr[1] = 0                                            # oflag
        attr[2] = termios.CS8 | termios.CREAD | termios.CLOCAL  # 8N1
        attr[3] = 0                                            # lflag: 无回显
        speed = getattr(termios, "B%d" % baud, None)
        if speed is None:
            raise SystemExit("termios 不支持 %d 波特率，试试 --baud 115200" % baud)
        attr[4] = speed
        attr[5] = speed
        attr[6][termios.VMIN] = 0
        attr[6][termios.VTIME] = 0
        termios.tcsetattr(self.fd, termios.TCSANOW, attr)
        termios.tcflush(self.fd, termios.TCIOFLUSH)

    def write(self, data):
        """非阻塞 fd：TX 满了就等一会儿再写，不丢帧"""
        view = memoryview(data)
        while view:
            try:
                n = os.write(self.fd, view)
                view = view[n:]
            except BlockingIOError:
                select.select([], [self.fd], [], 0.001)

    def read(self, timeout):
        r, _, _ = select.select([self.fd], [], [], timeout)
        if not r:
            return b""
        try:
            return os.read(self.fd, 4096)
        except BlockingIOError:
            return b""

    def close(self):
        os.close(self.fd)


# ------------------------------------------------------------------ 组帧 / 解析

def pack(ftype, seq, payload=b""):
    assert len(payload) <= MAX_PAYLOAD
    body = bytes([ftype, seq, len(payload)]) + payload
    return bytes([SOF1, SOF2]) + body + bytes([sum(body) & 0xFF, EOF1, EOF2])


def i16(v):
    return struct.pack("<h", v)


def u16(v):
    return struct.pack("<H", v)


def u32(v):
    return struct.pack("<I", v)


class Parser:
    """和固件一样：找 AA 55 → 读 LEN → 收满 8+LEN → 校验 SUM 和帧尾，失败只丢 1 字节"""

    def __init__(self):
        self.buf = bytearray()
        self.bad = 0

    def feed(self, data):
        self.buf += data
        out = []
        while True:
            while len(self.buf) >= 1 and self.buf[0] != SOF1:
                self.buf.pop(0)
            if len(self.buf) >= 2 and self.buf[1] != SOF2:
                self.buf.pop(0)
                continue
            if len(self.buf) < 5:
                break
            n = self.buf[4]
            if n > MAX_PAYLOAD:
                self.buf.pop(0)
                self.bad += 1
                continue
            total = 8 + n
            if len(self.buf) < total:
                break
            body = self.buf[2:5 + n]
            chk = sum(body) & 0xFF
            if chk != self.buf[5 + n] or self.buf[6 + n] != EOF1 or self.buf[7 + n] != EOF2:
                self.buf.pop(0)
                self.bad += 1
                continue
            out.append((self.buf[2], self.buf[3], bytes(self.buf[5:5 + n])))
            del self.buf[:total]
        return out


# ------------------------------------------------------------------ 打印

def decode(ftype, seq, pl):
    name = NAMES_DOWN.get(ftype, "0x%02X" % ftype)
    extra = ""
    if ftype == PONG and len(pl) >= 4:
        extra = "uptime=%d ms" % struct.unpack_from("<I", pl)[0]
    elif ftype == ACK and len(pl) >= 3:
        extra = "ack %s seq=%d → %s" % (NAMES_UP.get(pl[0], hex(pl[0])), pl[1],
                                        RESULT.get(pl[2], pl[2]))
    elif ftype == STATE and len(pl) >= 7:
        extra = "src=%s fault=0x%04X uptime=%d ms" % (
            SOURCE.get(pl[0], pl[0]), struct.unpack_from("<H", pl, 1)[0],
            struct.unpack_from("<I", pl, 3)[0])
    elif ftype == FIELD_STATUS and len(pl) >= 2:
        extra = "spawn=%d zone=%s" % (pl[0], "红" if pl[1] == 1 else "蓝")
    elif ftype == EVENT and len(pl) >= 2:
        extra = "code=%d data=%d%s" % (pl[0], pl[1], "（急停）" if pl[0] == 2 else "")
    elif ftype == MOVE_DONE and len(pl) >= 3:
        extra = "%s seq=%d → %s" % (NAMES_UP.get(pl[0], hex(pl[0])), pl[1],
                                    RESULT.get(pl[2], pl[2]))
    elif ftype == CMD_VEL and len(pl) >= 6:
        extra = "vx=%d vy=%d wz=%d" % struct.unpack_from("<hhh", pl)
    return "← %-12s seq=%-3d len=%-2d %-42s | %s" % (name, seq, len(pl), extra,
                                                     pl.hex(" ").upper())


class Link:
    def __init__(self, ser, verbose=True):
        self.ser = ser
        self.parser = Parser()
        self.seq = 0
        self.verbose = verbose
        self.pending = []   # 已收到但还没被 wait_for 取走的帧（同批里其它帧不能丢）

    def send(self, ftype, payload=b""):
        self.seq = (self.seq + 1) & 0xFF
        frame = pack(ftype, self.seq, payload)
        self.ser.write(frame)
        if self.verbose:
            print("→ %-12s seq=%-3d len=%-2d %s" % (NAMES_UP.get(ftype, hex(ftype)),
                                                    self.seq, len(payload),
                                                    payload.hex(" ").upper()))
        return self.seq

    def poll(self, timeout):
        """收一段时间的数据，全部进队列（并打印）"""
        deadline = time.time() + timeout
        while True:
            left = deadline - time.time()
            if left <= 0:
                return
            data = self.ser.read(left)
            if data:
                for f in self.parser.feed(data):
                    self.pending.append(f)
                    if len(self.pending) > 256:
                        self.pending.pop(0)
                    if self.verbose:
                        print(decode(*f))

    def wait_for(self, ftype, timeout=1.0, seq=None):
        """从队列里找某个 TYPE（可指定 seq）的帧，没有就继续收，不丢其它帧"""
        deadline = time.time() + timeout
        while True:
            for i, f in enumerate(self.pending):
                if f[0] == ftype and (seq is None or f[1] == seq):
                    del self.pending[:i + 1]
                    return f
            left = deadline - time.time()
            if left <= 0:
                return None
            self.poll(min(0.02, left))


# ------------------------------------------------------------------ 命令

def cmd_ping(link, args):
    for _ in range(args.count):
        t0 = time.time()
        seq = link.send(PING)
        f = link.wait_for(PONG, timeout=1.0, seq=None)
        if not f:
            print("  ✗ 1 s 内没收到 PONG")
            return 1
        uptime = struct.unpack_from("<I", f[2])[0]
        print("  ✓ 在线，uptime=%d ms，往返 %.1f ms" % (uptime, (time.time() - t0) * 1000))
        time.sleep(args.interval)
    return 0


def cmd_find(args):
    """逐个打开 /dev/ttyACM* /dev/ttyUSB*，发 PING 看谁回 PONG"""
    import glob
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("没找到串口设备")
        return 1
    found = []
    for p in ports:
        try:
            ser = Serial(p, args.baud)
        except Exception as e:                      # noqa: BLE001
            print("%-16s 打不开：%s" % (p, e))
            continue
        link = Link(ser, verbose=False)
        link.send(PING)
        pong = link.wait_for(PONG, timeout=0.6)
        ser.close()
        if pong:
            up = struct.unpack_from("<I", pong[2])[0]
            print("%-16s 有 PONG，uptime=%d ms  ← 就是它" % (p, up))
            found.append(p)
        else:
            print("%-16s 无应答" % p)
    return 0 if found else 1


def cmd_vel(link, args):
    pl = i16(args.vx) + i16(args.vy) + i16(args.wz)
    if args.sec:
        pl += u16(int(args.sec * 1000))
        seq = link.send(CMD_VEL_TIME, pl)
    else:
        seq = link.send(CMD_VEL, pl)
    ack = link.wait_for(ACK, timeout=1.0)
    if ack and ack[2][1] == seq:
        print("  ✓ ACK:", RESULT.get(ack[2][2], ack[2][2]))
    elif ack:
        print("  ! ACK 的 seq 对不上:", ack[2].hex(" "))

    if args.hz:
        period = 1.0 / args.hz
        end = time.time() + (args.sec or 1.0)   # 没给 --sec 就发 1 s
        while time.time() < end:
            link.send(CMD_VEL, pl)
            link.poll(period)
    return 0


def cmd_move(link, args):
    speed = max(5, min(100, args.speed))          # 与固件一致：5~100 cm/s
    seq = link.send(CMD_MOVE, i16(args.x) + i16(args.y) + u16(speed))
    ack = link.wait_for(ACK, timeout=1.0)
    if ack:
        print("  ✓ ACK:", RESULT.get(ack[2][2], ack[2][2]))
    timeout = max(args.timeout, (abs(args.x) ** 2 + abs(args.y) ** 2) ** 0.5 / speed + 2.0)
    done = link.wait_for(MOVE_DONE, timeout=timeout)
    if done:
        print("  ✓ MOVE_DONE:", RESULT.get(done[2][2], done[2][2]))
        return 0
    print("  ✗ %.1f s 内没有 MOVE_DONE" % timeout)
    return 1


def cmd_servo(link, args):
    lo, hi = (0, 2700) if args.wide else (SERVO_MIN, SERVO_MAX)
    if not (lo <= args.angle <= hi):
        print("  ✗ %d° 不在允许范围 %d~%d°（确实要越界请加 --wide）" % (
            args.angle / 10, lo / 10, hi / 10))
        return 2
    link.send(CMD_SERVO, bytes([args.id]) + u16(args.angle))
    ack = link.wait_for(ACK, timeout=1.0)
    print("  ✓ ACK:", RESULT.get(ack[2][2], ack[2][2]) if ack else "无响应")
    return 0


def cmd_servo_sweep(link, args):
    """在保守范围内慢速扫一遍：frm → to → frm，每步等舵机走完再下一步"""
    lo, hi = SERVO_MIN, SERVO_MAX
    a, b = args.frm, args.to
    if not (lo <= a <= hi and lo <= b <= hi):
        print("  ✗ 扫掠范围必须在 %d~%d°" % (lo / 10, hi / 10))
        return 2

    steps = list(range(a, b + 1, args.step)) + list(range(b - args.step, a - 1, -args.step))
    for i, ang in enumerate(steps, 1):
        link.send(CMD_SERVO, bytes([args.id]) + u16(ang))
        ack = link.wait_for(ACK, timeout=0.5)
        if ack is None or ack[2][2] != 0:
            print("  ✗ 第 %d 步 (%d°) 没有正常 ACK，停止" % (i, ang // 10))
            return 1
        print("  第 %2d/%d 步  %3d°" % (i, len(steps), ang // 10))
        time.sleep(args.interval)
    return 0


def cmd_estop(link, args):
    link.send(ESTOP, bytes([0]))
    got_ev = link.wait_for(EVENT, timeout=1.0)
    got_ack = link.wait_for(ACK, timeout=1.0)
    print("  EVENT:", "收到" if got_ev else "没收到",
          "| ACK:", "收到" if got_ack else "没收到")
    return 0 if (got_ev and got_ack) else 1


def cmd_field_ack(link, args):
    """上位机确认收到 FIELD_STATUS（0xAC），车会停止重发"""
    link.send(HOST_ACK, bytes([FIELD_STATUS, args.seq]))
    frames = link.poll(0.3)
    print("  ✓ 已发送 0xAC（acked_type=0x01, acked_seq=%d）" % args.seq)
    if link.pending:
        print("  ! 车回了帧，不符合预期：", link.pending)
        return 1
    print("  ✓ 车没有回帧")
    return 0


def cmd_monitor(link, args):
    print("监听中，Ctrl-C 退出（坏帧计数会随时打印）")
    try:
        while True:
            link.poll(0.2)
    except KeyboardInterrupt:
        print("\n坏帧 %d 个" % link.parser.bad)
    return 0


def cmd_selftest(link, args):
    fails = []

    def check(name, ok):
        print(("  ✓ " if ok else "  ✗ ") + name)
        if not ok:
            fails.append(name)

    print("[1] PING → PONG")
    seq = link.send(PING)
    pong = link.wait_for(PONG, timeout=1.0)
    check("收到 PONG", pong is not None)
    if pong:
        up1 = struct.unpack_from("<I", pong[2])[0]
        check("uptime > 0", up1 > 0)
        time.sleep(0.3)
        link.send(PING)
        pong2 = link.wait_for(PONG, timeout=1.0)
        check("两次 PING 之间 uptime 递增",
              pong2 is not None and struct.unpack_from("<I", pong2[2])[0] >= up1)

    print("[2] 未知类型 → ACK(UNSUPPORTED)")
    seq = link.send(0x42)
    ack = link.wait_for(ACK, timeout=1.0)
    check("收到 UNSUPPORTED", ack is not None and ack[2][2] == 2)

    print("[3] CMD_VEL 0（HOST 模式下静默接受，不拒时才有 ACK）")
    seq = link.send(CMD_VEL, i16(0) + i16(0) + i16(0))
    ack = link.wait_for(ACK, timeout=0.3)
    if ack is None:
        print("  ✓ 没有 ACK → 控制源是上位机，指令已直接执行")
    else:
        ok = ack[2][0] == CMD_VEL and ack[2][1] == seq and ack[2][2] == 3
        print(("  ✓ " if ok else "  ✗ ") + "回 ACK(REJECTED) → 当前不是上位机控制（CH5 没拨到上位机档）"
              if ok else "  ✗ 意外 ACK：%s" % ack[2].hex(" ").upper())
        if not ok:
            fails.append("CMD_VEL 的 ACK 异常")

    print("[4] ESTOP → EVENT + ACK")
    seq = link.send(ESTOP, bytes([0]))
    ev = link.wait_for(EVENT, timeout=1.0)
    ack = link.wait_for(ACK, timeout=1.0)
    check("收到 EVENT(code=2)", ev is not None and ev[2][0] == 2)
    check("收到 ACK 且回带 ESTOP/seq", ack is not None and ack[2][0] == ESTOP and ack[2][1] == seq)

    print("[5] 坏帧不影响后续解析（发 3 个随机字节再 PING）")
    link.ser.write(b"\xAA\x55\x7F\x00\x00")
    link.poll(0.1)
    link.send(PING)
    check("坏帧后仍能收到 PONG", link.wait_for(PONG, timeout=1.0) is not None)

    print()
    if fails:
        print("失败 %d 项：%s" % (len(fails), "，".join(fails)))
        return 1
    print("全部通过")
    return 0


# ------------------------------------------------------------------ main

def main():
    ap = argparse.ArgumentParser(description="上下位机串口协议测试")
    ap.add_argument("-p", "--port", default=None, help="串口，例如 /dev/ttyACM0（find 时可以不给）")
    ap.add_argument("-b", "--baud", type=int, default=921600)
    ap.add_argument("-q", "--quiet", action="store_true", help="只打结论，不逐帧打印")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("find", help="自动找哪个串口在跑协议").set_defaults(func=None)

    p = sub.add_parser("ping", help="判活")
    p.add_argument("--count", type=int, default=1)
    p.add_argument("--interval", type=float, default=1.0, help="两次之间间隔 s")
    p.set_defaults(func=cmd_ping)

    p = sub.add_parser("vel", help="速度指令")
    p.add_argument("vx", type=int)
    p.add_argument("vy", type=int)
    p.add_argument("wz", type=int)
    p.add_argument("--hz", type=float, default=0, help="持续以该频率连发")
    p.add_argument("--sec", type=float, default=0, help="带时长（CMD_VEL_TIME）或连发时长")
    p.set_defaults(func=cmd_vel)

    p = sub.add_parser("move", help="开到车体系某点")
    p.add_argument("x", type=int)
    p.add_argument("y", type=int)
    p.add_argument("--speed", type=int, default=40, help="cm/s，夹在 5~100（默认 40）")
    p.add_argument("--timeout", type=float, default=10.0)
    p.set_defaults(func=cmd_move)

    p = sub.add_parser("servo", help="舵机（默认限 100~170°）")
    p.add_argument("id", type=int, choices=(1, 2))
    p.add_argument("angle", type=int, help="0~2700 = 0~270°")
    p.add_argument("--wide", action="store_true", help="放开到 0~270°（危险）")
    p.set_defaults(func=cmd_servo)

    p = sub.add_parser("servo-sweep", help="在 100~170° 内慢速扫一遍")
    p.add_argument("id", type=int, choices=(1, 2))
    p.add_argument("--frm", type=int, default=SERVO_MIN)
    p.add_argument("--to", type=int, default=SERVO_MAX)
    p.add_argument("--step", type=int, default=50, help="每步多少 0.1°（默认 5°）")
    p.add_argument("--interval", type=float, default=0.4, help="每步停留秒数")
    p.set_defaults(func=cmd_servo_sweep)

    sub.add_parser("estop", help="急停").set_defaults(func=cmd_estop)
    p = sub.add_parser("field-ack", help="上位机确认场地信息（0xAC）")
    p.add_argument("--seq", type=int, default=0, help="收到的 FIELD_STATUS 的 SEQ")
    p.set_defaults(func=cmd_field_ack)
    sub.add_parser("monitor", help="只收不发的解析器").set_defaults(func=cmd_monitor)
    sub.add_parser("selftest", help="全流程自检").set_defaults(func=cmd_selftest)

    args = ap.parse_args()
    if args.cmd == "find":
        sys.exit(cmd_find(args))
    if not args.port:
        ap.error("除 find 外都要 -p 指定串口")

    ser = Serial(args.port, args.baud)
    link = Link(ser, verbose=not args.quiet)
    try:
        rc = args.func(link, args)
    finally:
        ser.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
