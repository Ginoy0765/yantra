"""Thread Turning G-code generator — G92 pass-by-pass with 3-step DOC + flank infeed.

External direction: tool starts near MAJOR dia, cuts INWARD to MINOR.
Internal direction: tool starts near MINOR dia (pilot), cuts OUTWARD to MAJOR.
Final cutting pass is forced to land at EXACT target diameter.
"""
import math


def fmt(v, d=3):
    """N22: trim trailing zeros - '15.6' not '15.600'; keep at least one decimal."""
    s = f'{v:.{d}f}'
    if '.' in s:
        s = s.rstrip('0').rstrip('.')
        if '.' not in s:
            s += '.0'
    return s


def _build_zone(cum_start, cum_end, doc_start, doc_end, zone_transition):
    """Return list of (doc, cum_after) passes that cover [cum_start, cum_end].
    U119 + U136: three transition modes:
      'linear'     - DOC ramps smoothly from doc_start down to doc_end.
                     Example: 0.10, 0.08, 0.07, 0.05 in one zone.
      'stepped'    - DOC is CONSTANT through the zone (= doc_end), with
                     the last pass shortened to land on cum_end exactly.
                     Example: 0.10, 0.10, 0.10, 0.05 (each zone flat).
      'degressive' - First-cut-anchored constant volume (Ginoy 2026-06):
                     pass 1 = the zone's First DOC (doc_start); each later
                     pass keeps the SAME chip cross-section, so cumulative
                     depth = doc_start*sqrt(i) and DOC of pass i =
                     doc_start*(sqrt(i) - sqrt(i-1)).  The zone's End DOC
                     (doc_end) is a FLOOR: once the shrinking DOC reaches
                     it, DOC holds constant so the pass count stays bounded.
    Last pass always lands exactly at cum_end."""
    depth = cum_end - cum_start
    if depth <= 1e-9:
        return []

    if zone_transition == 'stepped':
        # Constant DOC = doc_end throughout the zone.  Last pass
        # shortened to land exactly on the boundary.
        step = max(1e-6, float(doc_end))
        n = max(1, int(math.ceil(depth / step - 1e-9)))
        raw = [step] * n
        # Adjust last pass to exact boundary
        raw[-1] = depth - (n - 1) * step
        if raw[-1] <= 1e-9 and n > 1:
            # rounding artefact - merge with previous
            raw.pop()
            raw[-1] = depth - sum(raw[:-1])
    elif zone_transition == 'degressive':
        # First-cut-anchored constant volume: pass 1 = zone First DOC
        # (doc_start), each later pass keeps the same chip cross-section
        # (cumulative depth = d1*sqrt(i)).  End DOC (doc_end) is a FLOOR so
        # the pass count stays bounded.  Last pass trimmed to the boundary.
        d1 = max(1e-6, doc_start)
        floor = max(1e-6, doc_end)
        raw = []
        cum = 0.0
        i = 1
        while depth - cum > 1e-9 and len(raw) < 100000:
            doc_cv = d1 * (math.sqrt(i) - math.sqrt(i - 1))
            doc = doc_cv if doc_cv >= floor else floor
            if doc > depth - cum:
                doc = depth - cum
            raw.append(doc)
            cum += doc
            i += 1
    else:
        # Linear ramp (default 'linear' mode).  DOC ramps from
        # doc_start to doc_end over n passes; scaled so sum = depth.
        avg = (doc_start + doc_end) / 2.0
        n = max(1, int(round(depth / avg))) if avg > 0 else 1
        if n == 1:
            raw = [depth]
        else:
            raw = [doc_start - i * (doc_start - doc_end) / (n - 1)
                   for i in range(n)]
            s = sum(raw)
            if s > 0:
                raw = [r * depth / s for r in raw]

    out = []
    cum = cum_start
    for r in raw:
        cum += r
        out.append((r, cum))
    # Safety: force last cum to cum_end exactly (kills float drift)
    if out:
        prev = sum(d for d, _ in out[:-1])
        out[-1] = (cum_end - (cum_start + prev), cum_end)
    return out


def pass_schedule(thread_depth, first_doc, zones, zone_transition='linear'):
    """List of (doc, cum_after) cutting passes — the SAME schedule the
    generator produces (one _build_zone per zone).  Used by the live data
    panel so the Code Reference matches the G-code exactly."""
    out = []
    prev_pct = 0.0
    prev_doc = first_doc
    for pct, doc in zones:
        cum_start = thread_depth * prev_pct
        cum_end = thread_depth * pct
        for d, cum in _build_zone(cum_start, cum_end, prev_doc, doc, zone_transition):
            out.append((d, cum))
        prev_pct = pct
        prev_doc = doc
    return out


def generate(D, L, P, minor_dia=None, side='External', angle_deg=60, hand='RH',
             safe_x=1.0, safe_z=5.0, Vc=120,
             first_doc=0.05, end_doc_50=0.04, end_doc_80=0.03, end_doc_100=0.02,
             idle_passes=2,
             taper='None', dia_end_offset=0.0,
             infeed='Flank', flank_angle=None, flank_deg=None,
             zones=None, zone_transition='linear', num_starts=1,
             max_rpm=5000, start_angle=0.0):
    """Returns (program_string, total_time_sec, N_rpm, total_cutting_passes).

    U109: flank_angle is now THE infeed angle (auto from profile in UI =
    included/2 - 1° give-back, operator-overridable).  flank_deg kept as
    backward-compat: if only flank_deg is supplied, derive flank_angle the
    old way (= angle/2 - flank_deg) so legacy callers still work.

    U118: zones = configurable DOC zone pattern.  Format:
        [(pct_fraction, doc_at_end_mm), ...]   last entry MUST end at 1.0
    e.g. default 50/80/100: [(0.5, 0.04), (0.8, 0.03), (1.0, 0.02)]
         alt 40/60/90/100:  [(0.4, 0.04), (0.6, 0.035), (0.9, 0.025),
                             (1.0, 0.02)]
         alt 80/100:        [(0.8, 0.03), (1.0, 0.02)]
    If zones is None, falls back to the legacy 50/80/100 layout using
    end_doc_50, end_doc_80, end_doc_100."""
    if flank_angle is None:
        gb = 1.0 if flank_deg is None else float(flank_deg)
        flank_angle = (angle_deg / 2.0 - gb) if angle_deg > 0 else 0.0
    # U118: build zones list from legacy kwargs if not supplied.
    if zones is None:
        zones = [(0.5, end_doc_50), (0.8, end_doc_80), (1.0, end_doc_100)]
    # U124: multi-start threading.  Lead = N * P axial advance per spindle rev.
    # G92 F field uses LEAD (was P).  For each of the N starts, the full pass
    # set is emitted with a Q phase angle (in 0.001° units) so the controller
    # synchronises spindle phase to the correct start.
    num_starts = max(1, int(num_starts))
    lead = num_starts * P

    # U138: Respect Max RPM cap (default 5000) so emitted S value matches
    # what the UI displays for RPM.  Previously the G-code used the
    # uncapped theoretical RPM = Vc * 1000 / (pi * D).
    N_rpm = round(Vc * 1000 / (math.pi * D))
    if max_rpm and max_rpm > 0 and N_rpm > max_rpm:
        N_rpm = int(max_rpm)

    if minor_dia is None or minor_dia <= 0:
        minor_dia = (D - P) if side == 'Internal' else (D - 1.226 * P)

    thread_depth = (D - minor_dia) / 2

    # === N21 (B66): each zone ends EXACTLY at its target % of thread_depth ===
    # Zone 1: cum goes from 0    -> 50% of thread_depth (DOC ramps first_doc -> end_doc_50)
    # Zone 2: cum goes from 50%  -> 80%                 (DOC ramps end_doc_50 -> end_doc_80)
    # Zone 3: cum goes from 80%  -> 100%                (DOC ramps end_doc_80 -> end_doc_100)
    # We use the DOC values as caps for step size, not as literal step sizes.
    # Inside each zone, passes get progressively smaller but distributed so that
    # the LAST pass of the zone lands exactly on the boundary.

    def build_zone(cum_start, cum_end, doc_start, doc_end):
        # delegates to module-level _build_zone (single source of
        # truth, shared with pass_schedule() for the live panel).
        return _build_zone(cum_start, cum_end, doc_start, doc_end,
                           zone_transition)

    # U118: build passes from the configurable zones list.  Each entry of
    # `zones` is (pct_fraction, doc_at_end).  The DOC ramps from the
    # PREVIOUS zone's doc_end (or first_doc for the very first zone) down
    # to this zone's doc_end, with the last pass landing exactly on the
    # zone boundary.  Final entry's pct must be 1.0 (covers up to full
    # thread depth).
    passes = []
    prev_pct = 0.0
    prev_doc = first_doc
    for i, (pct, doc) in enumerate(zones):
        cum_start = thread_depth * prev_pct
        cum_end   = thread_depth * pct
        for d, cum in build_zone(cum_start, cum_end, prev_doc, doc):
            passes.append((f'STEP {i+1}', d, cum))
        prev_pct = pct
        prev_doc = doc

    total_cutting_passes = len(passes)

    # Idle/spring passes at final depth
    for _ in range(int(idle_passes)):
        passes.append(('SPRING', 0.0, thread_depth))

    # N4 / B60: dia_end_offset is the AUTHORITATIVE dia change over L
    # (positive = grows toward deep, negative = shrinks).
    # U33: External NPT positive, Internal NPT negative (fallback matches U33 sign).
    eff_end_offset = dia_end_offset
    if abs(eff_end_offset) < 1e-9 and (taper.startswith('NPT') or taper.startswith('BSPT')):
        eff_end_offset = (+L / 16.0) if side == 'External' else (-L / 16.0)

    # U38: G92 R must account for the full Z-travel (safe_z -> z_end = safe_z + L),
    # not just L. Otherwise the taper slope is stretched and the thread does not
    # match NPT 1:16 over the real thread length.
    # The dia_end_offset describes the diameter change from face (Z=0) to
    # Z_end=-L. G92 applies R over Z_span = safe_z - z_end = safe_z + L.
    # Scale: taper_R_emit = (dia_end_offset / 2) * Z_span / L, with Fanuc sign.
    # X_end is shifted by eff_end_offset so the tool reaches D_deep at Z=-L
    # (the user enters D = face dia, which stays unchanged in the header).
    #
    # Fanuc G92 convention: R = (X_start - X_end)/2 in radius form.
    #   External NPT (dia grows going deep) -> X_start < X_end -> R NEGATIVE.
    #   Internal NPT (dia shrinks going deep) -> X_start > X_end -> R POSITIVE.
    # Our eff_end_offset is positive when dia grows, so we flip the sign here.
    z_end_value = -L
    Z_span = safe_z - z_end_value   # positive, = safe_z + L
    if abs(eff_end_offset) > 1e-9 and L > 0:
        taper_R = -(eff_end_offset / 2.0) * (Z_span / L)   # Fanuc sign
        x_end_shift = eff_end_offset   # add to x_val so X_end = D_deep - 2cd
    else:
        taper_R = 0.0
        x_end_shift = 0.0
    R_str = f' R{fmt(taper_R, 4)}' if abs(taper_R) > 1e-6 else ''

    # Retract X
    retract_x = minor_dia - 0.5 * P if side == 'Internal' else D + 2 * safe_x

    # === FIX #62: L in header comment ===
    lines = []
    lines.append('%1001')
    lines.append(f'(M{D}X{P} L={L} {side})')
    lines.append('')
    # N51: only N1 appears once as the first G-code command; no other block numbers
    lines.append('N1 G40 G99')
    lines.append('G00 T0101 M08')
    # N42: spindle direction left as a bracketed comment so operator picks
    # M03 or M04 manually for the machine+tool+chuck combination.
    lines.append(f'G97 S{N_rpm} (M03/M04)')
    lines.append('')
    lines.append(f'G00 X{fmt(retract_x)} Z10.000')
    lines.append(f'G00 X{fmt(retract_x)} Z{fmt(safe_z)}')
    lines.append('')

    z_end = -L

    # U15/U17 + U109: 0 deg included angle = radial plunge (no Z shift).
    # Otherwise infeed angle = flank_angle (set by UI from profile, with
    # the standard 1° give-back already applied; operator can override).
    if angle_deg <= 0:
        infeed_ang_deg = 0.0
    else:
        infeed_ang_deg = max(0.0, float(flank_angle))
    tan_ang = math.tan(math.radians(infeed_ang_deg)) if infeed_ang_deg > 0 else 0.0

    # === Pass generation ===
    # External: X = D - 2*cum_depth (starts near MAJOR, decreases toward MINOR)
    # Internal: X = minor + 2*cum_depth (starts near MINOR, increases toward MAJOR)
    # N51: no per-pass block numbers; only N1 at top of program.
    # U124: outer loop = each thread start (1..N).  For multi-start, all
    # N starts use the same passes but with different Q (spindle phase).
    # Q is in 0.001° units (Fanuc convention): start k → Q = k * 360000 / N.
    # U159: ALWAYS emit Q (even Q0) to lock the thread phase to the spindle
    # index pulse.  This guarantees the thread restarts at the SAME angular
    # position on every G92 pass — even if RPM changes between passes,
    # because Q anchors threading to the spindle encoder index, not to time.
    # `start_angle` (deg) is added to each start's per-start phase offset
    # so the operator can shift the thread start angle as needed (rare —
    # default 0 = first start at index pulse).
    # F on G92 = LEAD (was P) for any N (= P for single-start, no change).
    f_field = fmt(lead)
    for start_idx in range(num_starts):
        # Per-start phase offset (degrees) + user-set base angle.
        per_start_deg = (start_idx * 360.0 / num_starts) + start_angle
        # Wrap to [0, 360) and convert to 0.001° units.
        per_start_deg = per_start_deg % 360.0
        q_phase = round(per_start_deg * 1000)
        q_str = f' Q{q_phase}'
        if num_starts > 1:
            lines.append('')
            lines.append(f'(--- START {start_idx + 1} OF {num_starts} ---)')
        for idx, (zone, doc, cd) in enumerate(passes):
            # Force the FINAL cutting pass X to exact target (extra safety)
            is_last_cut = (idx == total_cutting_passes - 1)
            cd_use = thread_depth if is_last_cut else cd

            # U38: shift X_end so the tool reaches the correct dia at Z_end
            # (D is the FACE dia; at Z_end the surface dia = D + eff_end_offset).
            if side == 'External':
                x_val = D - 2 * cd_use + x_end_shift
            else:
                x_val = minor_dia + 2 * cd_use + x_end_shift

            # Flank infeed Z shift per pass
            if infeed == 'Flank':
                z_start = safe_z - tan_ang * cd_use
            elif infeed == 'Alternate flank':
                z_start = safe_z - ((1 if idx % 2 == 0 else -1) * tan_ang * cd_use)
            else:   # Radial
                z_start = safe_z

            # N2/N51: no inline comments + no per-pass block numbers
            lines.append(f'G00 Z{fmt(z_start, 4)}')
            lines.append(f'G92 X{fmt(x_val)} Z{fmt(z_end)} F{f_field}{R_str}{q_str}')

    lines.append('')
    lines.append(f'G00 X{fmt(retract_x)} M09')
    lines.append(f'G00 Z{fmt(safe_z + 50)} M05')
    lines.append('M30')
    lines.append('%')

    # U124: multi-start runs the pass list N times, and feed per spindle
    # rev = LEAD (= N*P), so each pass takes L/(rpm*lead) rev.  Total
    # cutting time = N starts × len(passes) × L/(rpm*lead).
    time_per_pass = (L / (N_rpm * lead)) * 60 + 2
    total_time = 5 + num_starts * len(passes) * time_per_pass

    return '\n'.join(lines), total_time, N_rpm, total_cutting_passes