"""Annin HMI 6.0 program files (.ar4): plain text, CRLF, first line "##BEGINNING OF PROGRAM##".

Line formats used (copied from the HMI's own sample programs Home.ar4, Off Position.ar4, Stack Blocks.ar4):
  ##comment
  Move R [*] J1 <deg> J2 .. J6 <deg> J7 0.00 J8 0.00 J9 0.00 Sp <%> Ac <%> Dc <%> Rm <%> $ N
  Out On = <n>   /   Out Off = <n>
  Wait Time = <s>
Our own marker comments "##TC COUPLE <tool>" / "##TC RELEASE <tool>" tell hmi_check what the lock lines do;
the HMI ignores them like any other comment.
"""
import re
from dataclasses import dataclass, field

HEADER = "##BEGINNING OF PROGRAM##"
_MOVE_R = re.compile(r"^Move R \[\*\] J1 (\S+) J2 (\S+) J3 (\S+) J4 (\S+) J5 (\S+) J6 (\S+) .*?Sp (\S+) Ac (\S+) Dc (\S+) Rm (\S+)")


def move_r(q_deg, sp, ac, dc, rm):
    j = " ".join(f"J{i + 1} {round(float(v), 3) + 0.0:.3f}" for i, v in enumerate(q_deg))  # +0.0: no "-0.000"
    return f"Move R [*] {j} J7 0.00 J8 0.00 J9 0.00 Sp {sp:g} Ac {ac:g} Dc {dc:g} Rm {rm:g} $ N"


def out(on, n):
    return f"Out {'On' if on else 'Off'} = {n}"


def wait(s):
    return f"Wait Time = {s:g}"


def write(path, lines):
    with open(path, "w", newline="\r\n") as f:
        f.write("\n".join([HEADER] + list(lines)) + "\n")


@dataclass
class Op:
    kind: str                    # move | out | wait | couple | release | comment
    q_deg: list = field(default_factory=list)
    speed: float = 0.0
    value: float = 0.0
    on: bool = False
    tool: str = ""
    line_no: int = 0
    text: str = ""


def parse(path):
    """Parse the subset above. Other HMI commands are returned as kind='other' (not simulated)."""
    ops = []
    with open(path, newline="") as f:
        lines = f.read().replace("\r\n", "\n").split("\n")
    if not lines or lines[0].strip() != HEADER:
        raise ValueError(f"{path}: missing '{HEADER}' first line")
    for n, raw in enumerate(lines[1:], start=2):
        s = raw.strip()
        if not s:
            continue
        if s.startswith("##TC "):
            _, action, tool = s.split()[:3]
            ops.append(Op(kind=action.lower(), tool=tool, line_no=n, text=s))
        elif s.startswith("##"):
            ops.append(Op(kind="comment", line_no=n, text=s))
        elif m := _MOVE_R.match(s):
            v = [float(x) for x in m.groups()]
            ops.append(Op(kind="move", q_deg=v[:6], speed=v[6], line_no=n, text=s))
        elif m := re.match(r"^Out (On|Off) = (\d+)", s):
            ops.append(Op(kind="out", on=m.group(1) == "On", value=int(m.group(2)), line_no=n, text=s))
        elif m := re.match(r"^Wait Time = ([\d.]+)", s):
            ops.append(Op(kind="wait", value=float(m.group(1)), line_no=n, text=s))
        else:
            ops.append(Op(kind="other", line_no=n, text=s))
    return ops
