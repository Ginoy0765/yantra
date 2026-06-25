"""Web glue: replicate main.py post-processing around the engine modules.
Engine files (thread_mill/thread_turn/mill_post/axis_transform) are unchanged."""
import re
import thread_mill, mill_post, axis_transform

_EX = ('control', 'safez', 'sx', 'sy', 'sz', 'toolaxis', 'finishing')
_ORD = re.compile(r'^\((\d+)(?:ST|ND|RD|TH) PASS\)$')


def _gen_kwargs(p):
    return {k: p[k] for k in p if k not in _EX}


def _finishing(prog):
    """Before the final pass: spindle RPM x1.5, feed x0.3 (per operator spec)."""
    lines = prog.split('\n')
    passidx = [i for i, l in enumerate(lines) if _ORD.match(l.strip())]
    if not passidx:
        return prog
    rpm = None
    for l in lines:
        m = re.search(r'\bS(\d+)', l)
        if m:
            rpm = int(m.group(1)); break
    last = passidx[-1]
    if rpm:
        lines.insert(last, 'S%d' % int(round(rpm * 1.5)))
        last += 1
    end = len(lines)
    for i in range(last + 1, len(lines)):
        s = lines[i].strip()
        if _ORD.match(s) or s == 'M30' or s.startswith('G90 G00 Z'):
            end = i; break
    for i in range(last + 1, end):
        lines[i] = re.sub(r'\bF(\d+(?:\.\d*)?)',
                          lambda m: 'F%g' % (float(m.group(1)) * 0.3), lines[i])
    return '\n'.join(lines)


def post_mill(p):
    thread_mill.SAFE_Z = p['safez']
    prog = thread_mill.generate(**_gen_kwargs(p))
    prog = mill_post.format_for_control(prog, p['control'])
    if p.get('finishing'):
        prog = _finishing(prog)
    sx, sy, sz = p['sx'], p['sy'], p['sz']
    if sx or sy:
        prog = re.sub(r'(G00\s+)X0(?:\.0+)?\s+Y0(?:\.0+)?',
                      lambda m: m.group(1) + 'X' + thread_mill.fmt(sx) + ' Y' + thread_mill.fmt(sy),
                      prog, count=1)
    if sz:
        out = []
        absm = True
        for ln in prog.split('\n'):
            la = absm
            if 'G91' in ln:
                la = False; absm = False
            if 'G90' in ln:
                la = True; absm = True
            if la and 'Z' in ln:
                ln = re.sub(r'\bZ(-?\d+(?:\.\d*)?)',
                            lambda m: 'Z' + thread_mill.fmt(float(m.group(1)) + sz), ln)
            out.append(ln)
        prog = '\n'.join(out)
    ax = p['toolaxis']
    if ax and ax != 'Z-':
        try:
            prog = axis_transform.transform_program(prog, ax)
        except Exception:
            pass
    return prog


def mill_time(p):
    return thread_mill.cycle_time(**_gen_kwargs(p))
