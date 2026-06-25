"""U167: Tool axis transform — post-processor for milling G-code.

The base mill generator (`thread_mill.py`) emits G-code that assumes:
  - Bore axis = machine Z (cutter plunges in -Z direction)
  - Orbit plane = XY (G17)
  - Arc centers expressed as I (X-component) and J (Y-component)

For column mills with angle heads, Swiss machines, sub-spindles, etc.,
the cutter axis may be horizontal (X or Y) or inverted (Z+).  This
module rewrites the generated G-code to the new axis frame WITHOUT
touching the proven Z-down toolpath math in `thread_mill.py`.

DESIGN PRINCIPLES (per user, May 2026 — see engineering principles
memory):
  - PURE coordinate rotation, no hand-correction, no G02/G03 changes.
    Hand is controlled by the existing LH/RH selector in the UI.
    G02/G03 are left untouched — operator can edit if needed.
  - Default Z- mode produces output BYTE-IDENTICAL to before this
    feature existed.  Zero risk to machine-tested programs.
  - Verified against user's machine-proven programs:
      O151 G18 Y-, O151 G18 Y+, O151 G19 X-, O151 G19 X+
    plus M56x3 X- and Heidenhain X- variant.

TRANSFORM TABLE (locked 2026-05-XX):

  Tool axis  | new X | new Y | new Z | new I | new J | new K | Plane
  -----------+-------+-------+-------+-------+-------+-------+------
  Z- (def)   | X     | Y     | Z     | I     | J     | -     | G17
  Z+         | X     | Y     | -Z    | I     | J     | -     | G17
  X-         | -Z    | X     | Y     | -     | I     | J     | G19
  X+         | +Z    | X     | Y     | -     | I     | J     | G19
  Y-         | X     | -Z    | Y     | I     | -     | J     | G18
  Y+         | X     | +Z    | Y     | I     | -     | J     | G18

Z always stays as a lateral coordinate when possible — only its sign
flips for Z+.  For X/Y axes the original Z (depth) maps onto X or Y
respectively, with sign matching the tool axis direction.

This matches the user's actual proven programs, where the angle-head
cutter retains "vertical" Z motion as part of the orbit (cutter still
moves up/down through the workpiece even when the bore is horizontal).
"""

import re

# ---------- Public API ----------

VALID_AXES = ('Z-', 'Z+', 'X-', 'X+', 'Y-', 'Y+')


def transform_program(gcode: str, tool_axis: str = 'Z-') -> str:
    """Rewrite the generated G-code into the requested tool axis frame.

    Args:
        gcode: The full mill program text (multiple lines, '\n' separated).
        tool_axis: One of 'Z-', 'Z+', 'X-', 'X+', 'Y-', 'Y+'.
                   'Z-' is the default frame (matches `thread_mill.py`
                   output exactly — returns input unchanged).

    Returns:
        The transformed G-code text.  Same line count and structure;
        only coordinate addresses (X/Y/Z) and arc centers (I/J/K)
        are remapped per the transform table.  The plane code
        (G17/G18/G19) is updated on the line containing 'G40' setup.
    """
    if tool_axis is None or tool_axis == 'Z-':
        # Default frame — no transform.  Output identical to input.
        return gcode
    if tool_axis not in VALID_AXES:
        raise ValueError(
            f'Invalid tool_axis {tool_axis!r}; must be one of {VALID_AXES}')

    out_lines = []
    for line in gcode.splitlines():
        out_lines.append(_transform_line(line, tool_axis))
    return '\n'.join(out_lines) + ('\n' if gcode.endswith('\n') else '')


# ---------- Internal helpers ----------

# Regex for a G-code coordinate token: letter + signed number (incl. decimal).
# Captures the leading letter and the value separately so we can rewrite.
_TOKEN_RE = re.compile(
    r'(?<![A-Za-z])([XYZIJK])(-?\d+\.?\d*)',
    re.IGNORECASE,
)


def _transform_line(line: str, tool_axis: str) -> str:
    """Rewrite a single G-code line into the target tool-axis frame.

    Strategy: collect all (letter, value) pairs in original order,
    apply the transform mapping (which both renames the letter AND
    optionally negates the value), then substitute back into the line
    in-place.

    The G-plane code (G17/G18/G19) is updated on lines that already
    contain 'G40' (the start-of-program setup line in `thread_mill.py`).
    Other plane-code occurrences are left alone.
    """
    if not line.strip() or line.lstrip().startswith('('):
        # Comment or blank — pass through unchanged.
        return line

    # 1) Plane code update on the G54/G40 setup line.
    new_line = _maybe_inject_plane(line, tool_axis)

    # 1b) U247d: mirror transforms reverse G02↔G03 and G41↔G42
    # because the orbital-plane orientation flips.  Apply swap BEFORE
    # coordinate substitution so we operate on G-codes in their
    # original positions (substitution only touches X/Y/Z/I/J/K).
    if _IS_MIRROR.get(tool_axis, False):
        new_line = _swap_arc_and_comp(new_line)

    # 2) Coordinate substitution.
    #
    # We walk the line, collect each X/Y/Z/I/J/K token with its value,
    # transform them as a SET (because the rename may swap two letters
    # — e.g. X- swaps X and Z, so naive sequential rewrites would
    # collide).  Then rebuild the line by replacing each token's span.
    matches = list(_TOKEN_RE.finditer(new_line))
    if not matches:
        return new_line

    # Build the (letter -> value) dict for this line.
    by_letter = {}
    for m in matches:
        by_letter[m.group(1).upper()] = float(m.group(2))

    # Apply the transform to this dict.
    new_by_letter = _transform_addresses(by_letter, tool_axis)

    # Rebuild the line: replace each matched token in REVERSE order
    # (so earlier indices stay valid) with the transformed pair.
    chars = list(new_line)
    for m in reversed(matches):
        old_letter = m.group(1).upper()
        # Find which new letter (and value) this old letter became.
        # (Some letters may disappear entirely — e.g. I when the new
        # plane has no I.  In that case we drop the token.)
        new_letter, new_val = _emitted_for(old_letter, new_by_letter,
                                           tool_axis)
        if new_letter is None:
            # Drop the token (and any single trailing space).
            start, end = m.start(), m.end()
            # Trim one trailing space if present, to keep formatting tidy.
            if end < len(chars) and chars[end] == ' ':
                end += 1
            chars[start:end] = []
        else:
            replacement = f'{new_letter}{_fmt_val(new_val)}'
            chars[m.start():m.end()] = list(replacement)

    return ''.join(chars)


def _fmt_val(v: float) -> str:
    """Format a coordinate value the same way the original generator does.
    Use up to 4 decimals, strip trailing zeros, but keep a minimum of
    one decimal (so '5' stays '5.0' for clarity)."""
    if v == 0:
        return '0.0'
    s = f'{v:.4f}'.rstrip('0').rstrip('.')
    if '.' not in s:
        s += '.0'
    return s


# Plane code per axis.
_PLANE_CODE = {
    'Z-': 'G17', 'Z+': 'G17',
    'X-': 'G19', 'X+': 'G19',
    'Y-': 'G18', 'Y+': 'G18',
}

# U247d: which transforms are MIRRORS (det = -1)?  For mirror transforms,
# G02 ↔ G03 and G41 ↔ G42 must swap because the orientation of the
# orbital plane is reversed.  This list is computed from the coordinate
# mapping above:
#   Z-: identity                                 → det +1, no swap
#   Z+: Z sign flip                              → det -1, SWAP
#   X-: cyclic (X→Y, Y→Z, Z→+X)                  → det +1, no swap
#   X+: cyclic + Z flip (X→Y, Y→Z, Z→-X)         → det -1, SWAP
#   Y-: X→X, Y→Z, Z→+Y                           → det -1, SWAP
#   Y+: X→X, Y→Z, Z→-Y                           → det +1, no swap
# U-arcrule (Ginoy, 2026-06): the mirror-axis G02/G03 + G41/G42 swap is
# DISABLED.  Shop rule: arc direction follows hand + cut direction in
# EVERY tool axis — RH Top-to-Bottom = G02, RH Bottom-to-Top = G03 (LH
# mirrored) — and the cutter-comp side is left to match.  This restores
# this module's original design principle ("no G02/G03 changes"); the
# later U247d swap was wrongly flipping RH-TTB to G03 on Z+/X+/Y-.
# All False => _swap_arc_and_comp never runs (no arc/comp flip on any axis).
_IS_MIRROR = {
    'Z-': False, 'Z+': False,
    'X-': False, 'X+': False,
    'Y-': False, 'Y+': False,
}


def _swap_arc_and_comp(line: str) -> str:
    """For mirror transforms: swap G02↔G03 (arc direction) and
    G41↔G42 (cutter compensation side).  Same physical motion, new
    orientation requires opposite letter on these G-codes."""
    # Two-step swap using a placeholder to avoid double-substitution.
    line = re.sub(r'\bG02\b', '__G02_TMP__', line)
    line = re.sub(r'\bG03\b', 'G02', line)
    line = re.sub(r'__G02_TMP__', 'G03', line)
    line = re.sub(r'\bG41\b', '__G41_TMP__', line)
    line = re.sub(r'\bG42\b', 'G41', line)
    line = re.sub(r'__G41_TMP__', 'G42', line)
    return line


def _maybe_inject_plane(line: str, tool_axis: str) -> str:
    """If this line is the G40/G94 setup line that has 'G17' from the
    base generator, replace G17 with the appropriate plane code."""
    new_plane = _PLANE_CODE[tool_axis]
    if new_plane == 'G17':
        return line  # default — leave G17 as-is
    # Replace G17 only when paired with G40 / G94 (= the setup line).
    if 'G17' in line and 'G40' in line:
        return line.replace('G17', new_plane)
    return line


def _transform_addresses(by_letter: dict, tool_axis: str) -> dict:
    """Apply the transform table to a dict of {letter: value} for a line.

    Returns a NEW dict in the target frame.  The dict can have new
    letters (K added when the original had J in X-/Y- mappings) or
    drop letters that no longer apply.
    """
    X = by_letter.get('X')
    Y = by_letter.get('Y')
    Z = by_letter.get('Z')
    I = by_letter.get('I')
    J = by_letter.get('J')
    # K rarely present in default G17 output, but pass through if it is.
    K = by_letter.get('K')

    out = {}

    if tool_axis == 'Z+':
        # Just flip Z sign.  Everything else passes through.
        if X is not None: out['X'] = X
        if Y is not None: out['Y'] = Y
        if Z is not None: out['Z'] = -Z
        if I is not None: out['I'] = I
        if J is not None: out['J'] = J
        if K is not None: out['K'] = K
        return out

    if tool_axis == 'X-':
        # new X = -Z, new Y = X, new Z = Y
        # I (X-comp arc center) has no equivalent in G19 (no I) → DROP
        # J (Y-comp arc center) → new I-equiv is J? No — in G19,
        # the arc-center letters are J (Y) and K (Z).  So:
        #   old I (X-comp) → drop (G19 has no I-axis component for arc)
        #   old J (Y-comp) → new J (Y stays as Y in our mapping? no,
        #      old Y → new X, and we said arc J corresponds to Y axis,
        #      so when Y becomes new X, J no longer applies).
        # Re-derive arc mapping from coordinate mapping:
        #   The arc I/J/K refer to arc-center COMPONENTS along X/Y/Z
        #   IN THE ACTIVE PLANE.  For G19 (YZ plane), centers use J (Y)
        #   and K (Z).  Original code is in G17 (XY) using I and J.
        #
        # Coordinate mapping says original X-component maps to new Z,
        # original Y-component maps to new X, etc.  Arc centers follow
        # the same mapping:
        #   old I (= X-component of arc center) → new K? new ?
        # Actually simpler: arc center is a vector in 3D.  The
        # transformation rotates the vector.  In our X- mapping:
        #     old (Ix, Iy, 0) → new (-0, Ix, Iy) = (0, Ix, Iy)
        # So:
        #   - new X-component = 0 (no I emitted)
        #   - new Y-component = old I → emit as J
        #   - new Z-component = old J → emit as K
        # U247c: X- means tool tip points -X, retract direction is +X.
        # Z (original) keeps its sign when mapped to X (safe Z=+20 → X=+20,
        # cut Z=-depth → X=-depth).  Previous version negated, which
        # inverted both safe and cut positions.
        if X is not None: out['Y'] = X
        if Y is not None: out['Z'] = Y
        if Z is not None: out['X'] = +Z
        if I is not None: out['J'] = I
        if J is not None: out['K'] = J
        # K from input shouldn't normally exist in G17, ignore.
        return out

    if tool_axis == 'X+':
        # X+ means tool tip points +X, retract direction is -X.
        # Z (original) flips sign when mapped to X (safe Z=+20 → X=-20,
        # cut Z=-depth → X=+depth).
        if X is not None: out['Y'] = X
        if Y is not None: out['Z'] = Y
        if Z is not None: out['X'] = -Z
        if I is not None: out['J'] = I
        if J is not None: out['K'] = J
        return out

    if tool_axis == 'Y-':
        # U247c: Y- means tool tip points -Y, retract direction is +Y.
        # Z (original) keeps its sign when mapped to Y (safe Z=+20 → Y=+20,
        # cut Z=-depth → Y=-depth).  Previous version negated, which
        # inverted both safe and cut positions.
        # Plane G18 (XZ).  Arc center vector mapping:
        #   old (Ix, Iy, 0) → new (Ix, 0, Iy)
        #   - new X-component = old I → emit as I
        #   - new Y-component = 0 (no J emitted in G18)
        #   - new Z-component = old J → emit as K
        if X is not None: out['X'] = X
        if Y is not None: out['Z'] = Y
        if Z is not None: out['Y'] = +Z
        if I is not None: out['I'] = I
        if J is not None: out['K'] = J
        return out

    if tool_axis == 'Y+':
        # Y+ means tool tip points +Y, retract direction is -Y.
        # Z (original) flips sign when mapped to Y (safe Z=+20 → Y=-20,
        # cut Z=-depth → Y=+depth).
        if X is not None: out['X'] = X
        if Y is not None: out['Z'] = Y
        if Z is not None: out['Y'] = -Z
        if I is not None: out['I'] = I
        if J is not None: out['K'] = J
        return out

    # Fallback: no transform.
    return dict(by_letter)


# Map: for each ORIGINAL letter, return the (new_letter, new_value)
# in the transformed dict.  Used by `_transform_line` to do in-place
# token replacement.
def _emitted_for(orig_letter: str, new_by_letter: dict,
                 tool_axis: str):
    """Given an original letter that appeared in the source line, return
    (new_letter, new_value) so we can do in-place text replacement.

    Returns (None, None) if the original letter has no equivalent in the
    new frame (e.g., I when transformed to X-/Y- mapping where I drops).
    """
    # The mapping by axis tells us which new letter the old letter became.
    # Re-derived from the same logic as `_transform_addresses`, but
    # walking from old letter → new letter directly.
    if tool_axis in ('Z-',):
        return (orig_letter, new_by_letter.get(orig_letter))
    if tool_axis == 'Z+':
        if orig_letter == 'Z':
            return ('Z', new_by_letter.get('Z'))
        return (orig_letter, new_by_letter.get(orig_letter))
    if tool_axis in ('X-', 'X+'):
        # X→Y, Y→Z, Z→X; I→J, J→K
        old_to_new = {'X': 'Y', 'Y': 'Z', 'Z': 'X', 'I': 'J', 'J': 'K'}
        new_letter = old_to_new.get(orig_letter)
        if new_letter is None:
            return (None, None)
        return (new_letter, new_by_letter.get(new_letter))
    if tool_axis in ('Y-', 'Y+'):
        # X→X, Y→Z, Z→Y; I→I, J→K
        old_to_new = {'X': 'X', 'Y': 'Z', 'Z': 'Y', 'I': 'I', 'J': 'K'}
        new_letter = old_to_new.get(orig_letter)
        if new_letter is None:
            return (None, None)
        return (new_letter, new_by_letter.get(new_letter))
    return (orig_letter, new_by_letter.get(orig_letter))


# ---------- Self-test ----------

if __name__ == '__main__':
    sample = """%
O0001 (M22 P2 D0 d10 RH)
N1 M06 T1
G54
G90 G40 G17 G94
G00 X0.0 Y0 S1591 M03
G43 H1 Z20. M08
G01 G90 Z-7.1875 F5000
(1ST PASS)
G91 G01 G41 D1 X0.0 Y-3.5 Z0.0 F145
G03 X5.0 Y3.5 Z-0.3125 I1.275 J3.5 F36
G03 X0.0 Y0.0 Z-2.5 I-5.0 J0.0 F73
G03 X-5.0 Y3.5 Z-0.3125 I-3.725 J0.0
G01 G40 X-0.0 Y-3.5 F1000
G90 G00 Z20.
M30
%
"""
    print('--- Z- (default, no transform): ---')
    print(transform_program(sample, 'Z-'))
    print('--- X- transform: ---')
    print(transform_program(sample, 'X-'))
    print('--- Y- transform: ---')
    print(transform_program(sample, 'Y-'))
