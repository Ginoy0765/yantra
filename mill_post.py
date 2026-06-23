"""mill_post.py — milling control post-processor (YANTRA).

The milling generator + _apply_settings produce the program in the
**Fanuc I & J** dialect (tool-centre coords, G02/G03 with I/J arc centres,
G91 incremental threading, O-number, %).  That is the canonical/base
format.  Every other control is a transform of that base, so this module
takes the finished Fanuc program text and re-formats it for the selected
control.

Status:
  * 'Fanuc I & J'  — base (stamps its CONTROL identity line)            [done]
  * 'Haas'         — Fanuc body + HAAS identity line                    [done]
  * 'Fanuc R'      — I/J -> R; full 360 turn split into 2x180           [done]
  * 'Siemens I & J'— ;() comments, .SPF name, no %, T M06, G00 H1        [pending]
  * 'Siemens R'    — Siemens + CR= radius arcs (360 -> 2x180)           [pending]
  * 'Heidenhain'   — conversational L / CC / CP / IPA / RR rewrite       [pending]
  * 'Turnmill'     — handled upstream (separate generator)              [n/a]

Not-yet-built dialects fall through to the Fanuc base unchanged, so
selecting them never breaks output.
"""
import math
import re

CONTROLS = [
    'Fanuc I & J', 'Fanuc R', 'Siemens I & J', 'Siemens R',
    'Heidenhain', 'Haas', 'Turnmill',
]

# Identity comment stamped near the top of the program, one per control.
_IDENTITY = {
    'Fanuc I & J': '( CONTROL - FANUC I&J, TOOL CENTER COORDINATES, '
                   'SYSTEM UNIT 1 MM )',
    'Fanuc R':     '( CONTROL - FANUC R, TOOL CENTER COORDINATES, '
                   'SYSTEM UNIT 1 MM )',
    'Haas':        '( CONTROL - HAAS, TOOL CENTER COORDINATES, '
                   'SYSTEM UNIT 1 MM )',
}


# --------------------------------------------------------------------- #
# helpers
# --------------------------------------------------------------------- #
def _f(v):
    """Format a coordinate like the app: whole numbers as 'N.0', else
    trailing zeros trimmed."""
    v = round(float(v), 4)
    if abs(v) < 5e-5:
        return '0.0'
    s = ('%.4f' % v).rstrip('0').rstrip('.')
    if '.' not in s:
        s += '.0'
    return s


def _ff(v):
    """Format a feed: integer if whole (F87), else like _f."""
    v = float(v)
    if abs(v - round(v)) < 1e-6:
        return str(int(round(v)))
    return _f(v)


def _word(tok, words):
    """Float value of address `tok` (e.g. 'X') in a list of G-code words,
    or None if absent."""
    for w in words:
        if w and w[0] == tok and len(w) > 1 and (w[1].isdigit()
                                                 or w[1] in '+-.'):
            try:
                return float(w[1:])
            except ValueError:
                return None
    return None


def _insert_after_onumber(code, line):
    """Insert `line` right after the O-number line; if none, at the top."""
    out = code.split('\n')
    for i, ln in enumerate(out):
        s = ln.strip()
        if s.startswith('O') and any(c.isdigit() for c in s):
            out.insert(i + 1, line)
            return '\n'.join(out)
    out.insert(0, line)
    return '\n'.join(out)


def _insert_control_id(code, name):
    """Stamp '( CONTROL - <name> )' as the first line of the SUMMARY block
    (right after the '(--- SUMMARY ---)' marker).  `name` is the dropdown
    selection text only.  Comment style ( ) vs ;( ) is auto-detected;
    falls back to after the O-number if no summary marker is found."""
    out = code.split('\n')
    for i, ln in enumerate(out):
        u = ln.strip().upper()
        if u.startswith('(--- SUMMARY') or u.startswith(';(--- SUMMARY'):
            prefix = ';(' if u.startswith(';(') else '('
            out.insert(i + 1, '%s%s = %s)' % (prefix, 'CONTROL'.ljust(12), name))
            return '\n'.join(out)
    return _insert_after_onumber(code, '(%s = %s)' % ('CONTROL'.ljust(12), name))


# --------------------------------------------------------------------- #
# arc -> radius (Fanuc R / Siemens CR=)
# --------------------------------------------------------------------- #
def _arc_to_radius(line, radius_word):
    """Convert one G02/G03 I/J arc line to radius form.
    `radius_word` = 'R' (Fanuc) or 'CR=' (Siemens).
    A full 360 turn (no net XY move) is split into two 180 arcs.
    Non-arc lines pass through unchanged.  Returns a list of line(s)."""
    s = line.strip()
    up = s.upper()
    if 'G02' not in up and 'G03' not in up:
        return [line]
    words = s.split()
    I = _word('I', words)
    J = _word('J', words)
    if I is None or J is None:
        return [line]
    X = _word('X', words)
    Y = _word('Y', words)
    Z = _word('Z', words)
    F = _word('F', words)
    g02 = 'G02' in up
    gcode = 'G02' if g02 else 'G03'
    R = math.hypot(I, J)

    def emit(xx, yy, zz, ff, rsign):
        parts = [gcode, 'X' + _f(xx), 'Y' + _f(yy)]
        if zz is not None:
            parts.append('Z' + _f(zz))
        parts.append(radius_word + _f(rsign * R))
        if ff is not None:
            parts.append('F' + _ff(ff))
        return ' '.join(parts)

    full_circle = (X is not None and Y is not None
                   and abs(X) < 5e-5 and abs(Y) < 5e-5)
    if full_circle:
        zhalf = (Z / 2.0) if Z is not None else None
        return [emit(2 * I, 2 * J, zhalf, F, 1),
                emit(-2 * I, -2 * J, zhalf, None, 1)]

    # partial arc: choose R sign (+ for <=180 deg, - for >180 deg)
    a0 = math.atan2(-J, -I)
    ex = (X if X is not None else 0.0) - I
    ey = (Y if Y is not None else 0.0) - J
    a1 = math.atan2(ey, ex)
    sweep = ((a0 - a1) if g02 else (a1 - a0)) % (2 * math.pi)
    rsign = -1 if sweep > math.pi + 1e-6 else 1
    return [emit(X if X is not None else 0.0,
                 Y if Y is not None else 0.0, Z, F, rsign)]


def _convert_arcs(code, radius_word):
    """Convert every G02/G03 I/J arc in `code` to radius form
    (`radius_word` = 'R' Fanuc, 'CR=' Siemens); 360 turns -> 2x180."""
    out = []
    for ln in code.split('\n'):
        out.extend(_arc_to_radius(ln, radius_word))
    return '\n'.join(out)


def _to_radius_dialect(code, radius_word, identity):
    return _insert_after_onumber(_convert_arcs(code, radius_word), identity)


# --------------------------------------------------------------------- #
# Siemens 840D
# --------------------------------------------------------------------- #
def _siemens_common(code):
    """Shared Siemens transforms applied to a (possibly radius-converted)
    Fanuc-style program:
      * drop the % wrapper lines
      * O#### -> O####SPF
      * tool call  M06 T# -> T# M06
      * tool-length  G43 -> G00
      * comments  ( ... ) -> ;( ... )
    Arc body (I/J or CR=) is left as-is.  The CONTROL identity line is
    stamped separately by _insert_control_id (above the pitch comment)."""
    out = []
    for ln in code.split('\n'):
        if ln.strip() == '%':
            continue                                  # no % wrapper
        ln = re.sub(r'\bM06\b\s+(T\d+)\b', r'\1 M06', ln)   # T before M06
        ln = ln.replace('G43', 'G00')                 # length: G43 -> G00
        st = ln.strip()
        if re.match(r'^O\d+', st):                    # program number
            out.append(re.sub(r'^(\s*O\d+)', r'\1SPF', ln, count=1))
            continue
        if '(' in ln:                                 # comment -> ;( )
            ln = ln.replace('(', ';(', 1)
        out.append(ln)
    return '\n'.join(out)


def _to_siemens_ij(code):
    return _siemens_common(code)


def _to_siemens_r(code):
    return _siemens_common(_convert_arcs(code, 'CR='))


# --------------------------------------------------------------------- #
# Heidenhain (conversational / Klartext)
# --------------------------------------------------------------------- #
def _to_heidenhain(code):
    """Rewrite the Fanuc-base milling program into Heidenhain Klartext:
    BEGIN/END PGM wrapper, TOOL CALL, L rapids/feeds (FMAX = rapid),
    comp RR/RL/R0 (= G42/G41/G40), and arcs as CC (circle centre, incr.
    I/J) + CP IPA<sweep> IZ<helix> DR-/+ <comp>.  G91 moves use IX/IY/IZ,
    G90 moves use X/Y/Z.  STOP M02 + END PGM close the program."""
    lines = code.split('\n')

    # gather program number, tool number, spindle speed for the header
    prog_num, tool_num, spindle_s = '1', '1', None
    for ln in lines:
        s = ln.strip().upper()
        m = re.match(r'^O(\d+)', s)
        if m:
            prog_num = m.group(1)
        if 'M06' in s:
            mt = re.search(r'\bT(\d+)\b', s)
            if mt:
                tool_num = mt.group(1)
        ms = re.search(r'\bS(\d+)\b', s)
        if ms and spindle_s is None:
            spindle_s = ms.group(1)

    # CONTROL identity is stamped later by _insert_control_id (above pitch).
    out = ['BEGIN PGM %s MM' % prog_num]

    abs_mode = True
    comp = 'R0'

    for ln in lines:
        s = ln.strip()
        if not s or s == '%':
            continue
        up = s.upper()
        if s.startswith('('):                       # comment -> ;( )
            out.append(';' + s)
            continue
        if re.match(r'^O\d+', up):                  # program number (folded)
            continue
        if 'M06' in up:                             # tool change -> TOOL CALL
            tc = 'TOOL CALL %s Z' % tool_num
            if spindle_s:
                tc += ' S%s' % spindle_s
            out.append(tc)
            continue
        if up.startswith('G54'):                    # datum (omitted)
            continue
        if 'G17' in up and 'G94' in up:             # setup block (folded)
            abs_mode = 'G91' not in up
            continue

        words = up.split()
        if 'M30' in words or 'M02' in words:        # program end
            out.append('STOP M02')
            continue

        if 'G90' in up:
            abs_mode = True
        if 'G91' in up:
            abs_mode = False
        if 'G42' in up:
            comp = 'RR'
        elif 'G41' in up:
            comp = 'RL'
        elif 'G40' in up:
            comp = 'R0'

        X = _word('X', words); Y = _word('Y', words); Z = _word('Z', words)
        I = _word('I', words); J = _word('J', words); F = _word('F', words)
        mcodes = [w for w in words
                  if re.match(r'^M\d+$', w) and w != 'M06']
        m_str = ''.join(' ' + mc for mc in mcodes)

        is_arc = ('G02' in up or 'G03' in up)
        is_rapid = (('G00' in up or 'G43' in up) and not is_arc)

        if is_arc:
            g02 = 'G02' in up
            out.append('CC IX%s IY%s' % (_f(I or 0.0), _f(J or 0.0)))
            full = (X is not None and Y is not None
                    and abs(X) < 5e-5 and abs(Y) < 5e-5)
            if full:
                sweep_deg = 360.0
            else:
                a0 = math.atan2(-(J or 0.0), -(I or 0.0))
                a1 = math.atan2((Y or 0.0) - (J or 0.0),
                                (X or 0.0) - (I or 0.0))
                sw = ((a0 - a1) if g02 else (a1 - a0)) % (2 * math.pi)
                sweep_deg = math.degrees(sw)
            ipa = -sweep_deg if g02 else sweep_deg
            cp = 'CP IPA%s' % _f(ipa)
            if Z is not None:
                cp += ' IZ%s' % _f(Z)
            cp += ' %s %s' % ('DR-' if g02 else 'DR+', comp)
            if F is not None:
                cp += ' F%s' % _ff(F)
            out.append(cp + m_str)
            continue

        # linear / rapid -> L
        parts = ['L']
        if abs_mode:
            if X is not None:
                parts.append('X%s' % _f(X))
            if Y is not None:
                parts.append('Y%s' % _f(Y))
            if Z is not None:
                parts.append('Z%s' % _f(Z))
        else:
            if X is not None or Y is not None:
                parts.append('IX%s' % _f(X or 0.0))
                parts.append('IY%s' % _f(Y or 0.0))
                parts.append('IZ%s' % _f(Z or 0.0))
            elif Z is not None:
                parts.append('IZ%s' % _f(Z))
        # comp word only on contour moves (XY) or any feed; a pure-Z rapid
        # carries none (matches Heidenhain L Z.. FMAX positioning).
        if (X is not None or Y is not None) or not is_rapid:
            parts.append(comp)
        if is_rapid:
            parts.append('FMAX')
        elif F is not None:
            parts.append('F%s' % _ff(F))
        out.append(' '.join(parts) + m_str)

    out.append('END PGM %s MM' % prog_num)
    return '\n'.join(out)


# --------------------------------------------------------------------- #
# public entry point
# --------------------------------------------------------------------- #
def format_for_control(code, control):
    """Return `code` re-formatted for `control`, with a '( CONTROL - <name> )'
    line stamped above the header pitch comment (name = the dropdown
    selection only).  Unknown controls return the Fanuc base unchanged."""
    name = control or 'Fanuc I & J'
    if name in ('Fanuc I & J', 'Haas'):
        out = code
    elif name == 'Fanuc R':
        out = _convert_arcs(code, 'R')
    elif name == 'Siemens I & J':
        out = _to_siemens_ij(code)
    elif name == 'Siemens R':
        out = _to_siemens_r(code)
    elif name == 'Heidenhain':
        out = _to_heidenhain(code)
    else:
        return code            # Turnmill handled upstream / unknown
    return _insert_control_id(out, name)
