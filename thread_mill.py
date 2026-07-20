"""Thread milling G-code generator + cycle-time estimator.
Changes in v5: G90 absolute plunge, user-selectable spindle direction (M03/M04).
"""
import math

TP_FACTOR = 0.6134
SAFE_Z = 20
# U135: total per-program rapid budget (sec).  Covers G00 to start +
# initial G01 plunge at F5000 + final G00 retract.  Counted ONCE per
# program, regardless of pass count.  Does NOT include tool-change time
# (operator usually does 1 tool change for many threaded holes; counting
# tool-change here would overstate cycle time).
RAPID_TIME = 5


def fmt(v):
    s = f'{v:.4f}'.rstrip('0').rstrip('.')
    if '.' not in s:
        s += '.0'
    return s


def ord_pass(n):
    if 11 <= n % 100 <= 13:
        suf = 'TH'
    else:
        suf = {1: 'ST', 2: 'ND', 3: 'RD'}.get(n % 10, 'TH')
    return f'{n}{suf}'


def _build(D, L, P, d, Z, Vc, fz, hand, direction, mode,
           num_passes, pcts, repass_pitches, repass_offset,
           num_teeth, bottom_offset, spindle, dia_end_offset=0.0,
           minor=None, entry_style='45', num_starts=1,
           entry_feed_reduction=50.0, max_rpm=5000,
           stepped_style='Modified', is_external=False,
           strategy='Standard', entry_angle=0.0,
           tool_offset_mode='centre', rctf_strength=0.0):
    # U183: Tool-offset mode.
    #
    #   'centre' (default) — cutter dia is baked into orbit-radius math;
    #     orbit goes through the cutter centerline (D - d)/2 internal /
    #     (D + d)/2 external.  Machine offset register D1 must stay 0.
    #
    #   'od'     — orbit radius computed AS IF d=0 so the path lands on
    #     the thread contour itself.  Operator must set D1 offset =
    #     real cutter dia for runtime G41/G42 compensation.  Spindle
    #     RPM, fz, and safe-approach radius still use the real d
    #     (those are physical-machine quantities, not path geometry).
    d_path = 0.0 if tool_offset_mode == 'od' else d
    # v24.1 input validation — catch impossible geometry before math blows up
    if d <= 0 or D <= 0 or P <= 0 or L <= 0:
        raise ValueError(
            f"Invalid dimensions: D={D}, L={L}, P={P}, d={d}. All must be > 0."
        )
    if Vc <= 0 or fz <= 0 or Z <= 0:
        raise ValueError(
            f"Invalid cutting parameters: Vc={Vc}, fz={fz}, flutes Z={Z}. All must be > 0."
        )
    if not is_external and d >= D:
        raise ValueError(
            f"Tool diameter d={d} mm cannot be >= thread major diameter D={D} mm for internal threading. "
            f"The tool must fit inside the hole and orbit within it. "
            f"Try a tool of ~{max(2, int(D * 0.65))} mm or smaller."
        )
    # U138: Respect Max RPM cap (default 5000) when computing RPM and the
    # downstream lateral feed F1.  Previously the G-code used the uncapped
    # theoretical RPM = Vc * 1000 / (pi * d), so a small cutter (e.g. d=1.2)
    # at typical Vc would emit S15915 even when the UI displayed S5000.
    N_rpm = round(Vc * 1000 / (math.pi * d))
    if max_rpm and max_rpm > 0 and N_rpm > max_rpm:
        N_rpm = int(max_rpm)
    F1 = fz * Z * N_rpm
    # U147: n_teeth reduction is now baked into fz upstream (in main.py
    # on_material_change), so F1 here is already the reduced value.
    # No additional factor applied — fz IS the source of truth for
    # per-tooth chip load.
    P8 = P / 8
    # U124: multi-start threading.  lead = axial advance per spindle revolution
    # (= per orbit).  For single-start lead = P; for N-start lead = N*P.  The
    # main orbit and helix tracking use lead, while P stays as the cutter
    # tooth spacing (= axial pitch between adjacent thread crests).
    num_starts = max(1, int(num_starts))
    lead = num_starts * P

    # U46: thread depth = (D - minor) / 2, using whatever minor-dia rule is
    # active (internal 60 uses D-P, NPT uses D-1.6P, BSPT uses D-1.28P, etc).
    # Fallback to standard 60 external if no minor provided.
    if minor is not None and minor > 0 and minor < D:
        thread_depth = (D - minor) / 2.0
    else:
        thread_depth = TP_FACTOR * P   # legacy fallback

    is_bu = 1 if direction == 'Bottom to Top' else 0
    is_rh = 1 if hand == 'RH' else 0
    parity = (1 + is_rh + is_bu) % 2
    arc = 'G03' if parity == 1 else 'G02'
    comp = 'G41' if parity == 1 else 'G42'
    if is_external:  # external flips comp side vs internal: G03->G42, G02->G41
        comp = 'G42' if parity == 1 else 'G41'
    zs = 1 if is_bu else -1
    ys = 1 if parity == 1 else -1

    # U227: AUTO entry-style selection (internal only; external keeps its
    # own clean-entry logic).  Ginoy's rules:
    #   1. Top-to-Bottom helical -> ALWAYS 'Old'.
    #   2. else (BTT helical / stepped): if the 45-style comp-activation
    #      move's in-plane component would be < 0.2 mm -> 'Old' (45 starves
    #      G41/G42 pickup when the cutter nearly fills the bore; Old gives a
    #      bigger, readable move); else -> '45'.
    if entry_style == 'Auto':
        if is_external:
            entry_style = '45'
        elif mode == 'Helical' and not is_bu:
            entry_style = 'Old'          # TTB helical -> always Old
        else:
            _minor_eff = minor if (minor and 0 < minor < D) else (D - 2.0 * thread_depth)
            _gap = max(0.05, P / 10.0)
            _T = (_minor_eff / 2.0) if tool_offset_mode == 'od' else ((_minor_eff - d) / 2.0)
            _dist45 = max(0.0, _T - _gap)
            _act = _dist45 / (2.0 ** 0.5)   # 45 deg -> equal X and Y components
            entry_style = 'Old' if _act < 0.2 else '45'

    # U157: for EXTERNAL threading the cutter must approach from OUTSIDE
    # the workpiece — starting at bore center (X0 Y0) would crash through
    # a solid pipe / rod.  Compute a safe START radius equal to:
    #     safe_R = D_max/2 + d/2 + P     (= ~1 pitch beyond cutter outer
    #                                       edge when at biggest orbit)
    # where D_max accounts for taper growth (NPT/BSPT external grows
    # toward the body), so the cutter clears the largest part of the
    # workpiece + cutter + 1 pitch margin.  For internal: safe_R = 0
    # (start at bore center as before).
    if is_external:
        D_max_external = D + max(0.0, dia_end_offset)
        safe_R = D_max_external / 2.0 + d / 2.0 + 2 * P   # 2-pitch gap between thread OD and cutter
    else:
        safe_R = 0.0
    # U159: entry-arc feed helper.
    #   When RCTF > 0 AND entry style is 45° or 90°:
    #       Uses full-DOC ae (= same as orbit), so orbit_RCTF is already
    #       baked into F.  We add the axial-thinning factor for the
    #       style (45° -> 1/sin45 = 1.414; 90° -> 1.0), interpolated by
    #       rctf_strength, then CAP at 1.0× orbit feed per operator
    #       spec ("entry never exceeds orbit feed").
    #   Otherwise (RCTF = 0, or entry style = 'Old'):
    #       Legacy manual entry_feed_reduction% applies.
    SQRT2 = 2.0 ** 0.5
    def _entry_feed_for(F_orbit):
        if rctf_strength > 0 and entry_style in ('45', '90'):
            axial = SQRT2 if entry_style == '45' else 1.0
            effective = 1.0 + (axial - 1.0) * rctf_strength
            effective = min(1.0, effective)   # cap at 1.0× orbit feed
            return max(1.0, F_orbit * effective)
        feed_factor = max(0.05, 1.0 - max(0.0, min(95.0, entry_feed_reduction)) / 100.0)
        return max(1.0, F_orbit * feed_factor)

    def _ext_arc_geo(A):
        # External 45/90 single-point tangent entry/exit (base frame, pre-G68).
        # Standoff edge sits 1 pitch off major; entry arc is tangent to the orbit
        # at (A,0) AND passes through that standoff -> single point, no bridge.
        phi = math.radians(90.0 if entry_style == '90' else 45.0)
        cphi = math.cos(phi); sphi = math.sin(phi)
        s = safe_R                       # standoff radius = 1 pitch off MAJOR (D/2+d/2+P)
        Re = (A * A - 2 * A * s * cphi + s * s) / (2.0 * (A - s * cphi))
        # centre/baked mode (D=0): programmed = cutter centre, so plunge AT safe_R
        # (close standoff). OD/active-comp mode keeps the +d/2 offset.
        cr = safe_R if tool_offset_mode != 'od' else (safe_R + d / 2.0)
        e = -ys
        S = (s * cphi, e * s * sphi)
        eO = (cr * cphi, e * cr * sphi)
        exS = (s * cphi, ys * s * sphi)
        exO = (cr * cphi, ys * cr * sphi)
        return eO, S, Re, exS, exO

    # N23: header bracket includes cutter dia d and hand for quick operator check
    # N42: spindle direction left as a bracketed comment so operator picks
    # M03 or M04 manually for the machine+tool+chuck combination.
    lines = ['%', f'O0001 (D={D}  P={P}  L={L}  d={d}  {hand})', 'N1 M06 T1', 'G54',
             'G90 G40 G17 G94']
    # U159: entry angle — apply coordinate-system rotation around (0,0)
    # via G68 so the entire toolpath enters from any 0-360° direction
    # without changing every X/Y/I/J emission.  G69 cancels at end.
    # Default 0 = no rotation (G68/G69 lines suppressed).
    use_g68 = abs(entry_angle) > 1e-6
    if use_g68 and num_starts == 1:
        lines.append(f'G68 X0 Y0 R{fmt(entry_angle)}   ( ENTRY ANGLE = {fmt(entry_angle)} DEG )')
    # External 45/90: go STRAIGHT to the diagonal plunge corner (no
    # redundant (safe_R,0) pre-position).  eO is A-independent (cr=safe_R).
    _ext_clean = is_external and entry_style in ('45', '90') and mode == 'Helical'
    _start_xy = _ext_arc_geo(D / 2.0)[0] if _ext_clean else (safe_R, 0.0)
    lines.append(f'G00 X{fmt(_start_xy[0])} Y{fmt(_start_xy[1])} S{N_rpm} (M03/M04)')
    lines.append(f'G43 H1 Z{SAFE_Z}. M08')

    # U157: track XY position too — needed for external entry/exit which
    # starts cutter at the standoff/plunge position instead of bore center.
    st = {'z': SAFE_Z, 'p': 0, 'time': RAPID_TIME, 'first_plunge': True,
          'x': _start_xy[0], 'y': _start_xy[1]}

    def plunge_to(tz):
        # U47: first plunge stays G90 absolute as the editable anchor.
        # All subsequent plunges are G91 incremental from the previous tool Z,
        # so if the operator edits the anchor Z the whole program shifts with it.
        # U84: track the ACTUAL emitted Z (after fmt rounding) so the tracker
        # always equals the machine's real position, not theoretical math.
        # U135: NO time added per plunge - all rapids covered by the single
        # RAPID_TIME budget at the start of the program.  Adding per-plunge
        # time would overcount on multi-pass and on multi-hole PCD programs.
        if st['first_plunge']:
            lines.append(f'G01 G90 Z{fmt(tz)} F5000')
            st['z'] = float(fmt(tz))    # what the machine actually plunges to
            st['first_plunge'] = False
        else:
            dz = tz - st['z']
            lines.append(f'G01 G91 Z{fmt(dz)} F5000')
            st['z'] += float(fmt(dz))   # what the machine actually moves

    def phdr():
        st['p'] += 1
        lines.append(f'({ord_pass(st["p"])} PASS)')

    def emit_entry(A, feed_mult_override=None):
        """U123 + U127 + U130 + U132 + U133: emit linear approach + tangent
        entry arc to orbit start (A, 0).  Three styles based on `entry_style`:
          '45'  - linear diagonal at 45° (short ~64-90° arc).
          '90'  - linear along ±Y axis (medium ~127° arc).
          'Old' - legacy A_h × √2 diagonal with NO auto-clamp.  Original
                  behaviour from before U127; can scrape the bore wall on
                  tight setups.  Provided for backward compatibility.
        U133: gap from bore wall reverted to P/10 (10% of pitch).  P/2
        from U132 was too aggressive vs thread height (~0.61·P) - left
        the cutter using less than half the bore for engagement.
        U157: for EXTERNAL threads, the cutter starts at (safe_R, 0)
        OUTSIDE the workpiece (per the modified initial G00).  Approach
        is a single linear radial move INWARD from (safe_R, 0) to
        (A, 0) with Z descent matching what an entry arc would do.
        No tangent arc (sharp corner at orbit start, accepted with
        reduced entry feed).  Cutter never crosses the workpiece axis,
        so it never tries to come through the workpiece centre."""
        A_h = A / 2
        # U219: apply the orbit feed-multiplier to the entry-arc and
        # linear-approach feeds too, so the operator sees a SINGLE
        # consistent feed value for each pass instead of one for the
        # entry arc and a different one for the orbit.  The multiplier
        # is supplied by continuous_helix() which knows the per-pass
        # feed_mults from U218.
        _mult = (feed_mult_override
                 if feed_mult_override is not None else 1.0)
        F = F1 * (2 * A) / D * _mult
        SQRT2 = 2.0 ** 0.5
        # U157: external entry.  45/90 (helical) -> 1-pitch-gap standoff +
        # single tangent arc onto the orbit start.  Else legacy radial inward.
        if is_external:
            entry_feed = max(1, round(_entry_feed_for(F)))
            if entry_style in ('45', '90') and mode == 'Helical':
                # CLEAN entry (no roll-in arc): single straight move from the
                # clear plunge position straight to the orbit start (A, 0).
                dxe = A - st['x']; dye = 0.0 - st['y']
                lines.append(f'G91 G01 {comp} D1 X{fmt(dxe)} Y{fmt(dye)} '
                             f'Z{fmt(zs*P8)} F{entry_feed}')
                st['x'] = A; st['y'] = 0.0
                return
            dx_ext = A - st['x']
            dy_ext = 0.0 - st['y']
            lines.append(f'G91 G01 {comp} D1 X{fmt(dx_ext)} Y{fmt(dy_ext)} '
                         f'Z{fmt(zs*P8)} F{entry_feed}')
            st['x'] = A
            st['y'] = 0.0
            return
        # Per-style direction of the linear approach.
        if entry_style == '90':
            unit_dx, unit_dy = 0.0, -ys * 1.0
        else:   # '45' or 'Old' (both use 45° diagonal direction)
            inv_root2 = 1.0 / SQRT2
            unit_dx, unit_dy = inv_root2, -ys * inv_root2
        # U133: gap from wall = P/10 (10% of pitch).
        gap = max(0.05, P / 10.0)
        # U203: in Tool OD mode, the programmed position is at the thread
        # contour and the controller offsets the actual cutter centerline
        # INWARD by d/2 via G42.  So the PROGRAMMED ly can safely go all
        # the way to (minor/2 − gap) — the bore radius minus a safety gap
        # — and the actual cutter still stays inside the bore.  This
        # gives a much larger entry-arc radius (always > cutter_radius +
        # 0.2P), eliminating the alarm-or-linear-substitute fallback for
        # tight-bore-vs-cutter setups.
        is_od_mode = (tool_offset_mode == 'od')
        if entry_style == 'Old':
            # Legacy: A_h × √2 diagonal endpoint, no auto-clamp.
            default_dist = A_h * SQRT2
            apply_clamp = False
        elif minor is not None and minor > 0 and minor < D:
            if is_od_mode:
                # OD mode: programmed = thread contour, cutter offset
                # inward by d/2 → can program almost to bore wall.
                T = minor / 2.0
            else:
                # Centerline mode: programmed = cutter centerline,
                # limit to cutter-edge clearance from bore wall.
                T = (minor - d) / 2.0
            default_dist = max(0.0, T - gap)
            apply_clamp = True
        else:
            # External threading: legacy A_h-based default with no clamp.
            default_dist = A_h * SQRT2 if entry_style == '45' else A_h
            apply_clamp = False
        # Auto-clamp safety net (skipped for 'Old' style by design).
        dist = default_dist
        if apply_clamp and minor is not None and minor > 0 and minor < D:
            if is_od_mode:
                T = minor / 2.0
            else:
                T = (minor - d) / 2.0
            R_max = max(0.0, T - gap)
            if dist > R_max:
                dist = R_max
        # Linear endpoint (incremental from origin = bore center)
        lx = unit_dx * dist
        ly = unit_dy * dist
        # U128 / U164 / U215: linear pre-positioning gets a TWO-PHASE
        # feed schedule when the approach is reasonably long (> 1 mm):
        #
        #   - First 80% of the move at DOUBLE the orbit feed (2 × F)
        #     — cutter is in clear air, no material engagement, so we
        #     save cycle time by ripping through fast.
        #   - Last 20% at the normal orbit feed (F) — gives the
        #     servos time to settle before the entry arc engages.
        #
        # For very short approaches (≤ 1 mm) we keep a single move at
        # the normal feed (no benefit splitting; just adds an extra
        # block).  G42 cutter compensation is activated on the first
        # block — Fanuc ramps the offset across that move.
        fast_feed = round(2 * F)
        slow_feed = round(F)
        # 'Old' style enters in ONE lead-in move to match Carmex
        # (G91 G42 D1 X.. Y.. in a single block); the 80/20 fast/slow
        # split is a cycle-time optimisation kept only for 45/90 styles.
        if dist > 1.0 and entry_style != 'Old':
            lx_fast = lx * 0.8; ly_fast = ly * 0.8
            lx_slow = lx - lx_fast; ly_slow = ly - ly_fast
            lines.append(
                f'G91 G01 {comp} D1 X{fmt(lx_fast)} Y{fmt(ly_fast)} '
                f'Z0.0 F{fast_feed}')
            lines.append(
                f'G01 X{fmt(lx_slow)} Y{fmt(ly_slow)} Z0.0 F{slow_feed}')
        else:
            lines.append(
                f'G91 G01 {comp} D1 X{fmt(lx)} Y{fmt(ly)} Z0.0 F{slow_feed}')
        # Compute arc tangent at (A, 0), passing through (lx, ly).
        # Center on +X axis at (cx, 0):
        #   cx = (lx² + ly² - A²) / (2*(lx - A))
        #   r = A - cx
        #   Incremental I = cx - lx, J = -ly.
        if abs(lx - A) < 1e-9:
            # Degenerate: linear endpoint at orbit X.  Skip arc (just at orbit).
            return
        cx = (lx * lx + ly * ly - A * A) / (2.0 * (lx - A))
        r = A - cx
        I_inc = cx - lx
        J_inc = -ly
        # U128 / U159: entry arc feed.
        # When RCTF = 0 (or Old style): legacy F * (1 - entry_feed_reduction%/100).
        # When RCTF > 0 and style is 45/90: axial-thinning factor with cap
        # at 1.0× orbit feed.  All handled by the _entry_feed_for helper.
        entry_feed = max(1, round(_entry_feed_for(F)))
        # U196 / U198: in Tool OD mode the controller applies G42 cutter
        # compensation at runtime.  Compensating an arc whose radius is
        # smaller than (or barely bigger than) the cutter radius is
        # geometrically risky — Fanuc alarm 041 fires when the offset
        # arc would self-intersect.  Per user (U198) the rule is:
        #
        #     entry_radius_min = cutter_radius + 0.2 × pitch
        #
        # i.e. a safety cushion of 0.2P above bare cutter radius.
        # Example: 6 mm cutter (R=3) at 1 mm pitch → r_min = 3.2 mm.
        # When the computed tangent-arc radius is below this threshold,
        # substitute a direct G01 linear move; controller ramps the
        # comp during the straight segment without alarming.
        r_min_safe = d / 2.0 + 0.2 * P
        if tool_offset_mode == 'od' and abs(r) < r_min_safe:
            lines.append(
                '(* OD-MODE ENTRY ARC < CUTTER_R + 0.2P - SUBSTITUTED LINEAR *)')
            lines.append(
                f'(  programmed_r={r:.3f} cutter_r={d/2:.3f} '
                f'r_min={r_min_safe:.3f} mm)')
            lines.append(
                f'G01 X{fmt(A - lx)} Y{fmt(-ly)} Z{fmt(zs*P8)} F{entry_feed}')
        else:
            lines.append(f'{arc} X{fmt(A - lx)} Y{fmt(-ly)} Z{fmt(zs*P8)} '
                         f'I{fmt(I_inc)} J{fmt(J_inc)} F{entry_feed}')

    def emit_exit(A_end, feed_mult_override=None):
        """U123 + U127 + U130 + U132 + U133: emit tangent exit arc + linear
        retract.  Mirrors emit_entry across the X-axis.  Same default_dist
        policy: T - P/10 for internal threading; legacy A_h × √2 with no
        clamp for 'Old' style and for external threading.
        U157: for EXTERNAL threads, retract is a single linear radial
        move OUTWARD from (A_end, 0) back to (safe_R, 0) with Z step
        matching what an exit arc would do.  Cutter ends OUTSIDE the
        workpiece, ready for the next axial pass or final retract."""
        A_h_end = A_end / 2.0
        # U219: same per-pass multiplier as emit_entry, supplied by
        # continuous_helix() so the exit-arc + retract feeds match the
        # orbit feed for that pass.
        _mult = (feed_mult_override
                 if feed_mult_override is not None else 1.0)
        F = F1 * (2 * A_end) / D * _mult
        SQRT2 = 2.0 ** 0.5
        # U157: external exit.  45/90 (helical) -> tangent arc out + comp-off
        # radial to the outer point.  Else legacy radial outward.
        if is_external:
            if entry_style in ('45', '90') and mode == 'Helical':
                # CLEAN exit (no roll-out arc): single straight G40 move from
                # the orbit end straight out to the clear exit point.
                _eO, _S, Re, exS, exO = _ext_arc_geo(A_end)
                dxx = exO[0] - A_end; dyy = exO[1] - 0.0
                lines.append(f'G91 G01 G40 X{fmt(dxx)} Y{fmt(dyy)} '
                             f'Z{fmt(zs*P8)} F1000')
                st['x'] = exO[0]; st['y'] = exO[1]
                return
            dx_ext = safe_R - st['x']
            dy_ext = 0.0 - st['y']
            lines.append(f'G91 G01 G40 X{fmt(dx_ext)} Y{fmt(dy_ext)} '
                         f'Z{fmt(zs*P8)} F1000')
            st['x'] = safe_R
            st['y'] = 0.0
            return
        # Direction mirrors emit_entry across X-axis (Y sign flips).
        if entry_style == '90':
            unit_dx, unit_dy = 0.0, ys * 1.0
        else:   # '45' or 'Old'
            inv_root2 = 1.0 / SQRT2
            unit_dx, unit_dy = inv_root2, ys * inv_root2
        gap = max(0.05, P / 10.0)
        # U203: same OD-mode wider-clearance rule as emit_entry.
        is_od_mode = (tool_offset_mode == 'od')
        if entry_style == 'Old':
            default_dist = A_h_end * SQRT2
            apply_clamp = False
        elif minor is not None and minor > 0 and minor < D:
            T = (minor / 2.0) if is_od_mode else ((minor - d) / 2.0)
            default_dist = max(0.0, T - gap)
            apply_clamp = True
        else:
            default_dist = A_h_end * SQRT2 if entry_style == '45' else A_h_end
            apply_clamp = False
        dist = default_dist
        if apply_clamp and minor is not None and minor > 0 and minor < D:
            T = (minor / 2.0) if is_od_mode else ((minor - d) / 2.0)
            R_max = max(0.0, T - gap)
            if dist > R_max:
                dist = R_max
        # Linear endpoint absolute position (cutter ends here after exit linear)
        ex = unit_dx * dist
        ey = unit_dy * dist
        if abs(ex - A_end) < 1e-9:
            return
        # Arc from (A_end, 0) back to (ex, ey), tangent at (A_end, 0).
        cx = (ex * ex + ey * ey - A_end * A_end) / (2.0 * (ex - A_end))
        r_exit = A_end - cx
        # Incremental: dx = ex - A_end, dy = ey - 0, I = cx - A_end, J = 0
        dx_arc = ex - A_end
        dy_arc = ey
        I_inc = cx - A_end
        J_inc = 0.0
        # U196 / U198: same Fanuc-041 guard as emit_entry — substitute
        # linear when exit-arc radius < cutter_radius + 0.2P safety
        # cushion (per user-defined formula).
        r_min_safe_exit = d / 2.0 + 0.2 * P
        if tool_offset_mode == 'od' and abs(r_exit) < r_min_safe_exit:
            lines.append(
                '(* OD-MODE EXIT ARC < CUTTER_R + 0.2P - SUBSTITUTED LINEAR *)')
            lines.append(
                f'G01 X{fmt(dx_arc)} Y{fmt(dy_arc)} Z{fmt(zs*P8)}')
        else:
            lines.append(f'{arc} X{fmt(dx_arc)} Y{fmt(dy_arc)} Z{fmt(zs*P8)} '
                         f'I{fmt(I_inc)} J{fmt(J_inc)}')
        # Linear retract back to origin
        # Old style: exit G40 runs on the previous modal feed (no F word)
        # per Ginoy; other styles keep the explicit retract feed.
        if entry_style == 'Old':
            lines.append(f'G01 G40 X{fmt(-ex)} Y{fmt(-ey)}')
        else:
            lines.append(f'G01 G40 X{fmt(-ex)} Y{fmt(-ey)} F1000')

    def continuous_helix(A, n_orbits, plunge_z, dia_taper=0.0, feed_multiplier=1.0):
        """Helical thread pass. If dia_taper != 0, emit 4 quarter-arcs per orbit
        with progressive radial shrink = (dia_taper/2) spread over n_orbits.
        U150: feed_multiplier > 1 doubles the orbit feed for spring/cleanup
        passes (subsequent 100% passes after the first one).  Entry arc and
        exit arc keep their base feed (computed in emit_entry / emit_exit
        independently)."""
        A_h = A / 2
        F = F1 * (2 * A) / D * feed_multiplier   # U150: orbit feed × multiplier
        if is_external and entry_style in ('45', '90'):
            eO = _ext_arc_geo(A)[0]
            if st['z'] < SAFE_Z - 1e-9:
                lines.append(f'G90 G00 Z{SAFE_Z}.')
                st['z'] = SAFE_Z
            if abs(st['x'] - eO[0]) > 1e-6 or abs(st['y'] - eO[1]) > 1e-6:
                lines.append(f'G90 G00 X{fmt(eO[0])} Y{fmt(eO[1])}')
                st['x'], st['y'] = eO[0], eO[1]
        plunge_to(plunge_z)
        phdr()
        # U219: pass the same per-pass feed_multiplier into emit_entry
        # so the linear approach, entry arc, and orbit feeds all match.
        emit_entry(A, feed_mult_override=feed_multiplier)
        if abs(dia_taper) < 1e-9 or n_orbits < 1:
            # straight thread - single I/J per orbit as before
            # U124: orbit Z descent = lead (= num_starts * P) per turn.
            for i in range(n_orbits):
                if i == 0:
                    lines.append(f'{arc} X0.0 Y0.0 Z{fmt(zs*lead)} I{fmt(-A)} J0.0 F{round(F)}')
                else:
                    lines.append(f'{arc} X0.0 Y0.0 Z{fmt(zs*lead)} I{fmt(-A)} J0.0')
            R_end = A
        else:
            # N4: tapered helix - 4 quarter-arcs per orbit, progressive radius change.
            # U43: direction of R change depends on helix direction (is_bu) and sign
            # of dia_taper. Over the whole helix, R goes from R_start to R_end where:
            #   R_end - R_start = -zs * (dia_taper / 2)
            # U124: per-quarter Z descent = lead/4 (was P/4) for multi-start.
            dR_per_quarter = -zs * dia_taper / (n_orbits * 8)
            dz_q = zs * (lead / 4.0)
            R_cur = A
            feed_set = False
            for q in range(n_orbits * 4):
                R_nxt = R_cur + dR_per_quarter
                quad = q % 4
                if arc == 'G03':   # CCW: +X -> +Y -> -X -> -Y -> +X
                    if quad == 0:   dX, dY, I, J = -R_cur, +R_nxt,  -R_cur, 0
                    elif quad == 1: dX, dY, I, J = -R_nxt, -R_cur,  0,      -R_cur
                    elif quad == 2: dX, dY, I, J = +R_cur, -R_nxt,  +R_cur, 0
                    else:           dX, dY, I, J = +R_nxt, +R_cur,  0,      +R_cur
                else:              # G02 CW: +X -> -Y -> -X -> +Y -> +X
                    # U48: fixed signs on quads 1 and 3.
                    if quad == 0:   dX, dY, I, J = -R_cur, -R_nxt,  -R_cur, 0
                    elif quad == 1: dX, dY, I, J = -R_nxt, +R_cur,  0,      +R_cur
                    elif quad == 2: dX, dY, I, J = +R_cur, +R_nxt,  +R_cur, 0
                    else:           dX, dY, I, J = +R_nxt, -R_cur,  0,      -R_cur
                feed_tag = f' F{round(F)}' if not feed_set else ''
                lines.append(f'{arc} X{fmt(dX)} Y{fmt(dY)} Z{fmt(dz_q)} I{fmt(I)} J{fmt(J)}{feed_tag}')
                feed_set = True
                R_cur = R_nxt
            R_end = R_cur
        # U49: exit arc + line use R_end so tool lands exactly at origin (0,0)
        # regardless of how taper shifted the orbit radius. Prevents inter-pass drift.
        # U123: exit geometry is style-dependent; helper handles all three.
        # U219: pass per-pass feed_multiplier so exit-arc + retract feeds
        # match the orbit feed for this pass.
        emit_exit(R_end, feed_mult_override=feed_multiplier)
        # U82: track ACTUAL emitted Z values (after fmt() rounding) so the
        # next pass's incremental return-to-start move corrects for any
        # micron-level drift caused by P/8 not rounding cleanly to 4 decimals.
        # Without this, pitches like 25.4/12 = 2.1167 mm would accumulate ~1
        # micron drift per pass.
        q_emit = float(fmt(zs * P8))         # quarter-arc Z (lead-in / lead-out)
        if abs(dia_taper) < 1e-9 or n_orbits < 1:
            # Straight helix: 2 quarter arcs + n_orbits full orbits at zs*lead
            # U124: orbit emits lead (was P).
            full_emit = float(fmt(zs * lead))
            actual_dz = 2 * q_emit + n_orbits * full_emit
        else:
            # Tapered helix: 2 quarter arcs + (n_orbits*4) quarters at zs*lead/4
            # U124: quarter arc emits lead/4 (was P/4).
            quad_emit = float(fmt(zs * (lead / 4.0)))
            actual_dz = 2 * q_emit + n_orbits * 4 * quad_emit
        st['z'] = plunge_z + actual_dz
        # U140: precise cycle-time accounting (was 2πA·(n+1)/F).
        # Components per pass:
        #   orbit:  n_orbits full circles at F          = (2π·A·n_orbits) / F
        #   entry:  ~quarter circle at reduced feed     = (π·A/2) / (F·feed_factor)
        #   exit:   ~quarter circle at full feed F      = (π·A/2) / F
        # Quarter-circle is an approximation - actual entry arc sweep
        # depends on entry style (45° / 90° / Old) but quarter circle
        # is a reasonable mid-point.
        # U159: entry_feed via the same RCTF-aware helper so cycle time
        # matches the actual emitted feed.
        entry_feed = max(1.0, _entry_feed_for(F))
        st['time'] += (
            (2 * math.pi * A * n_orbits / F) * 60        # main orbits
            + (math.pi * A / 2 / entry_feed) * 60         # entry arc
            + (math.pi * A / 2 / F) * 60                  # exit arc
        )

    def stepped_orbit(A, dia_per_orbit=0.0, feed_multiplier=1.0,
                      emit_header=True):
        """U149: dia_per_orbit > 0 -> emit 4 quarter-arcs with progressive R
        (matches helical-tapered precision, eliminates within-orbit cone
        stair-step for NPT/BSPT).  dia_per_orbit == 0 -> single G02/G03
        full circle (legacy behaviour for straight threads).
        U150: feed_multiplier doubles orbit feed for subsequent 100% passes."""
        A_h = A / 2
        F = F1 * (2 * A) / D * feed_multiplier   # U150
        # U227: in stepped mode one radial pass may be cut in several
        # axial bands (when L > cutter coverage).  Only the FIRST band
        # emits the (Nth PASS) header so the program counts RADIAL
        # step-overs as passes, not axial step-downs.  emit_header is
        # set False by stepped_sequence for the 2nd+ bands.
        if emit_header:
            phdr()
        # U219: pass per-pass feed_multiplier so stepped-mode entry
        # arc + linear approach feeds match the orbit feed.
        emit_entry(A, feed_mult_override=feed_multiplier)
        # U124: orbit Z descent = lead (was P) for multi-start support.
        if abs(dia_per_orbit) < 1e-9:
            # Straight thread: legacy single full-circle orbit.
            lines.append(f'{arc} X0.0 Y0.0 Z{fmt(zs*lead)} I{fmt(-A)} J0.0 F{round(F)}')
            R_end = A
        else:
            # U149: tapered thread -> 4 quarter-arcs per orbit with
            # progressive radius.  Per-quarter dR = (radial change per
            # orbit) / 4 = (dia_per_orbit / 2) / 4 = dia_per_orbit / 8.
            # Sign convention: zs = -1 for TTB (descending), so radial
            # increases (for outward-growing taper) over the descent
            # → matches helical's `dR_per_quarter = -zs * dia_taper / (n_orbits * 8)`.
            dR_per_quarter = -zs * dia_per_orbit / 8.0
            dz_q = zs * (lead / 4.0)
            R_cur = A
            feed_set = False
            for q in range(4):
                R_nxt = R_cur + dR_per_quarter
                quad = q % 4
                if arc == 'G03':   # CCW: +X -> +Y -> -X -> -Y -> +X
                    if quad == 0:   dX, dY, I, J = -R_cur, +R_nxt,  -R_cur, 0
                    elif quad == 1: dX, dY, I, J = -R_nxt, -R_cur,  0,      -R_cur
                    elif quad == 2: dX, dY, I, J = +R_cur, -R_nxt,  +R_cur, 0
                    else:           dX, dY, I, J = +R_nxt, +R_cur,  0,      +R_cur
                else:              # G02 CW
                    if quad == 0:   dX, dY, I, J = -R_cur, -R_nxt,  -R_cur, 0
                    elif quad == 1: dX, dY, I, J = -R_nxt, +R_cur,  0,      +R_cur
                    elif quad == 2: dX, dY, I, J = +R_cur, +R_nxt,  +R_cur, 0
                    else:           dX, dY, I, J = +R_nxt, -R_cur,  0,      -R_cur
                feed_tag = f' F{round(F)}' if not feed_set else ''
                lines.append(f'{arc} X{fmt(dX)} Y{fmt(dY)} Z{fmt(dz_q)} I{fmt(I)} J{fmt(J)}{feed_tag}')
                feed_set = True
                R_cur = R_nxt
            R_end = R_cur
        # U219: same per-pass multiplier for the stepped-mode exit.
        emit_exit(R_end, feed_mult_override=feed_multiplier)
        # U82/U124: track ACTUAL emitted Z (after fmt rounding).
        q_emit = float(fmt(zs * P8))
        full_emit = float(fmt(zs * lead))
        st['z'] += 2 * q_emit + full_emit
        # U140: precise per-pass time = orbit + entry arc + exit arc.
        feed_factor = max(0.05, 1.0 - max(0.0, min(95.0, entry_feed_reduction)) / 100.0)
        entry_feed = max(1.0, F * feed_factor)
        st['time'] += (
            (2 * math.pi * A / F) * 60                    # 1 orbit at F
            + (math.pi * A / 2 / entry_feed) * 60         # entry arc
            + (math.pi * A / 2 / F) * 60                  # exit arc
        )

    def stepped_sequence(A_base, plunge_zs, taper=0.0, feed_multiplier=1.0,
                         top_first=True):
        """U122: now takes a LIST of plunge_zs (one per orbit) instead of
        a single first_plunge_z + implicit step pattern.  This lets the
        caller customise the LAST pass's plunge_z (e.g. to make it match
        thread depth, while middle passes follow the natural max-engagement
        step pattern).  Backward compat: pass [first_plunge_z] for the
        single-orbit case.
        U149: per-orbit dia change for tapered threads is computed inside
        from the closure-accessed `taper` and `L`, then passed to
        stepped_orbit so each orbit's 4 quarter-arcs match the cone slope.
        U150: feed_multiplier propagates to every stepped_orbit call so
        subsequent 100% radial passes get doubled orbit feed.
        U154: per-orbit radius computed from the orbit's ACTUAL Z position
        (not from a pass-index fraction).  For tapered threads (NPT etc.)
        this matches the bore radius at each pass's depth, regardless of
        pass order (Modified top-first or Carmex bottom-first) or whether
        L is much larger than the cutter's coverage.  Earlier (U153) used
        oi/(n-1) which assumed pass 0 was AT the face / -L extreme; this
        broke for L >> coverage where pass 0 sits well below the face.
        The top_first arg is kept for callsite compatibility but no longer
        affects the math."""
        n_orbits = len(plunge_zs)
        if n_orbits == 0:
            return
        plunge_to(plunge_zs[0])
        for oi in range(n_orbits):
            # U154: compute per-orbit radius from this orbit's ACTUAL Z
            # (not pass index).  orbit_start_z = plunge_z + zs*P/8 (after
            # entry).  frac_z = -orbit_start_z / L → 0 at face, 1 at -L.
            # A = A_base (face radius) + frac_z * taper/2.  For NPT
            # internal taper < 0 → A shrinks as cutter goes deeper,
            # matching the narrowing bore.
            if L > 1e-9:
                orbit_start_z = plunge_zs[oi] + zs * P8
                frac_z = max(0.0, min(1.0, -orbit_start_z / L))
            else:
                frac_z = 0.0
            A = A_base + frac_z * taper / 2
            if oi > 0:
                # Reposition cutter Z to plunge_zs[oi] before this orbit.
                target_z = plunge_zs[oi]
                dz = target_z - st['z']
                if abs(dz) > 1e-6:
                    lines.append(f'G01 G91 Z{fmt(dz)} F5000')
                    st['z'] += float(fmt(dz))
            # U149: per-orbit diametrical change for tapered threads.
            # Within ONE orbit (axial advance = lead), the cone diameter
            # changes by lead × (taper / L).  Pass this to stepped_orbit
            # so it emits 4 quarter-arcs with progressive radius.
            if abs(taper) > 1e-9 and L > 0:
                dia_per_orbit = lead * taper / L
            else:
                dia_per_orbit = 0.0
            stepped_orbit(A, dia_per_orbit=dia_per_orbit,
                          feed_multiplier=feed_multiplier,
                          emit_header=(oi == 0))

    # U150: detect "subsequent 100% pass" — the FIRST 100% pass uses base
    # feed (it's doing real cutting); SUBSEQUENT 100% passes get the
    # orbit feed doubled (they're spring/cleanup passes, removing little
    # to no material).  Build a parallel `feed_mults` list aligned with
    # pcts.  Entry/exit arcs always keep base feed.
    feed_mults = []
    seen_100 = False
    for pct in pcts:
        if pct >= 100:
            feed_mults.append(2.0 if seen_100 else 1.0)
            seen_100 = True
        else:
            feed_mults.append(1.0)

    # U221: depth-aware feed taper for n > 5 passes — calibrated
    # against user's specification.
    #
    # User stated targets (consistent across D=18 and D=20 example
    # programs):
    #   pass 1 (shallow)  = 181 mm/min
    #   pass N (full)     = 103 mm/min
    #
    # Both programs had F1 (= F peripheral = fz × Z × RPM) ≈ 790, so
    # the targets translate to:
    #   pass N feed = F1 × 0.130
    #   pass 1 feed = F1 × 0.229  (= pass N × 1.75)
    #
    # Linear interpolation between these two endpoints gives:
    #   desired_F(i) = F1 * 0.130 * (1.75 - 0.75 * i/(N-1))
    #
    # Convert to feed_mults so continuous_helix's F = F1*2A/D*mult
    # produces the desired_F:
    #   feed_mults[i] = desired_F(i) * D / (F1 * 2 * A_i)
    F1_PASS_N_FACTOR = 0.130    # pass N feed as a fraction of F1
    if len(pcts) > 5:
        N = len(pcts)
        for i in range(N):
            pct_i = pcts[i]
            A_i = (D - d_path) / 2.0 - (1.0 - pct_i / 100.0) * thread_depth
            if abs(A_i) < 1e-9 or F1 <= 0:
                continue
            taper_i = 1.75 - 0.75 * (i / (N - 1)) if N > 1 else 1.0
            desired_F = F1 * F1_PASS_N_FACTOR * taper_i
            F_geom_i = F1 * 2.0 * A_i / D
            if F_geom_i <= 0:
                continue
            new_mult = desired_F / F_geom_i
            # Don't override U150 spring × 2 (subsequent 100% passes
            # are intentional cleanup; let them stay fast).
            if feed_mults[i] >= 1.99:
                continue
            feed_mults[i] = new_mult

    # U158: Finishing strategy — last pass gets surface-quality treatment.
    # Identify last radial pass (= last pcts entry with pct > 0).  Before
    # that pass, emit S{round(N_rpm * 1.25)} to bump RPM (Vc × 125%).  For
    # that pass's orbit feed, override feed_mult to 0.5 (= half feed).
    # Other strategies (Productivity / Weak setup / Tool life) are handled
    # via Vc/fz pre-scaling in main.py before _build is called.
    last_radial_idx = -1
    for i in range(len(pcts)):
        if pcts[i] > 0:
            last_radial_idx = i
    is_finishing = (strategy == 'Finishing' and last_radial_idx >= 0)
    if is_finishing:
        # Compute Finishing RPM (capped at max_rpm).
        finishing_rpm = round(N_rpm * 1.25)
        if max_rpm and max_rpm > 0 and finishing_rpm > max_rpm:
            finishing_rpm = int(max_rpm)
        # Override feed_mult for last pass to 0.5 (overrides U150 spring×2
        # if both would apply — Finishing's surface-quality intent wins).
        feed_mults[last_radial_idx] = 0.5

    # U157: per-pass Radial Chip Thinning Factor (RCTF) compensation.
    # Activated by the 'RCTF comp (%)' slider in Thread Milling (only
    # visible in Constant-volume Path mode).  strength = slider/100:
    #   0.0  -> no compensation (base fz unchanged)
    #   1.0  -> full RCTF math, feed *= RCTF_i per pass
    #   0.5  -> halfway, feed *= 1 + (RCTF_i - 1) * 0.5
    # ae = INCREMENTAL cut depth for this pass (new material only).
    #   For ID:  ae = (pct_i - pct_prev) / 100 × thread_depth
    #   For OD:  same formula (incremental cuts are symmetric).
    # Pure math: no rubbing-floor warning, no edge-max clamp, no per-pass
    # log lines.  Stacks on top of all earlier feed_mults adjustments.
    if rctf_strength > 0 and d > 0:
        prev_pct = 0.0
        for i in range(len(pcts)):
            this_pct = pcts[i]
            if this_pct <= 0:
                continue
            doc_i = (this_pct - prev_pct) / 100.0 * thread_depth
            prev_pct = this_pct
            if doc_i <= 0:
                # Spring / cleanup pass — no new material; RCTF undefined.
                continue
            ae_over_dc = doc_i / d
            if ae_over_dc >= 0.5:
                rctf_i = 1.0
            else:
                denom = 1.0 - (1.0 - 2.0 * ae_over_dc) ** 2
                rctf_i = 1.0 / math.sqrt(denom) if denom > 0 else 1.0
            # Interpolate by strength:  pure math at 1.0, identity at 0.0.
            effective = 1.0 + (rctf_i - 1.0) * rctf_strength
            feed_mults[i] *= effective

    # U226: spring-pass feed tied to the 100% DOC pass feed.
    #   "100% DOC pass" = FIRST pass reaching pct >= 100 (cuts to full
    #   depth).  Spring/cleanup passes = any SUBSEQUENT pct >= 100 pass.
    #   Rule (per operator):
    #     RCTF applied (rctf_strength > 0)  -> spring feed = 100% DOC pass
    #         feed (SAME multiplier).  The DOC pass already runs fast from
    #         the RCTF boost, so doubling on top would be too aggressive.
    #     RCTF not applied (= 0)            -> spring feed = 2 x 100% DOC
    #         pass feed (no new material removed, safe to rip through fast).
    #   Runs AFTER U157 so m_doc reflects any RCTF boost on the DOC pass.
    #   Finishing's 0.5x on the last radial pass WINS (surface quality).
    #   Cycle time follows automatically: continuous_helix / stepped_orbit
    #   compute both emitted feed and the time component from this mult.
    doc_idx = next((i for i, p in enumerate(pcts) if p >= 100), None)
    if doc_idx is not None:
        m_doc = feed_mults[doc_idx]
        spring_mult = m_doc if rctf_strength > 0 else 2.0 * m_doc
        for i in range(doc_idx + 1, len(pcts)):
            if pcts[i] >= 100:
                if is_finishing and i == last_radial_idx:
                    continue  # Finishing surface-quality override wins
                feed_mults[i] = spring_mult

    if mode == 'Helical':
        # U124: orbit count and plunge_z use lead (= num_starts * P) per orbit.
        n_z = math.ceil(L / lead)
        # U155: plunge_z compensates for emitted-Z rounding so cutter
        # orbit-end lands at exactly -L (TTB) or orbit-start at -L (BTT).
        # For tapered helical, each quad arc emits Z = fmt(zs * lead/4).
        # When lead/4 doesn't round cleanly to 4 decimals (e.g. NPT
        # P=1.8143 → P/4 = 0.453575 rounds to 0.4536), the rounding
        # accumulates to a few microns over many orbits.  Pre-compute
        # the actual emitted descent and adjust plunge_z to compensate.
        is_tapered = abs(dia_end_offset) > 1e-9
        if is_bu:
            # BTT: cutter_b at orbit start (after entry) = -L exactly.
            # plunge_z = -L - P/8 (entry ascends by P/8 emit which is
            # exact since fmt(P/8) is exact for typical P values).
            plunge_z = -(L + P8)
        else:
            # TTB: cutter_b at orbit end = -L exactly.
            if is_tapered:
                # Tapered helix uses 4 quad arcs per orbit, each at lead/4.
                entry_emit = float(fmt(-P8))                  # zs=-1
                quad_emit = float(fmt(-lead / 4.0))
                # plunge_z + entry_emit + n_z*4*quad_emit = -L
                plunge_z = -L - entry_emit - n_z * 4 * quad_emit
            else:
                # Straight helix uses 1 full circle per orbit at lead.
                full_emit = float(fmt(-lead))                  # zs=-1
                plunge_z = -L - float(fmt(-P8)) - n_z * full_emit
        _cut_idx = len(lines)
        for p_idx in range(num_passes):
            pct = pcts[p_idx]
            if pct <= 0:
                continue
            # A_face = orbit radius at the workpiece FACE (Z=0).
            # U46 (internal):  A = (D - d)/2 - (1 - pct/100) * thread_depth
            #                  cutter axis INSIDE bore.  pct=100 → cutter
            #                  outer at major; pct=0 → cutter outer at minor.
            # U157 (external): A = (D + d)/2 - (pct/100) * thread_depth
            #                  cutter axis OUTSIDE workpiece.  pct=100 →
            #                  cutter inner at minor; pct=0 → cutter inner
            #                  at major (no cut, just touches workpiece OD).
            if is_external:
                A_face = (D + d_path) / 2 - (pct / 100) * thread_depth
            else:
                A_face = (D - d_path) / 2 - (1 - pct / 100) * thread_depth
            # U154: for tapered helical (NPT/BSPT/Manual), the orbit radius
            # varies axially.  TTB starts the helix at the FACE (top)
            # so A_helix_start = A_face.  BTT starts the helix at the
            # DEEPEST point Z=-L where the bore radius is different —
            # for NPT internal the bore is narrower at the bottom, so
            # A_helix_start_BTT = A_face + dia_end_offset/2.  Without
            # this correction the BTT cutter starts 0.9 mm OUTSIDE the
            # bore wall and would crash.
            if is_bu and abs(dia_end_offset) > 1e-9:
                A_helix_start = A_face + dia_end_offset / 2.0
            else:
                A_helix_start = A_face
            # U158: emit S code BEFORE the last pass for Finishing strategy.
            # Spindle speed bumps to 1.25× before the cutter starts the
            # final radial pass — gives the spindle time to spin up before
            # cutting begins.
            if is_finishing and p_idx == last_radial_idx:
                lines.append(f'S{finishing_rpm}')
            continuous_helix(A_helix_start, n_z, plunge_z,
                             dia_taper=dia_end_offset,
                             feed_multiplier=feed_mults[p_idx])
        # U6: run bottom repass whenever pitches > 0 (offset=0 is valid, not disabled)
        # U124: repass plunge uses lead per orbit too.
        # U154: for tapered threads the repass is at the BOTTOM of the
        # thread, where the bore is narrower (NPT internal).  Use bottom
        # radius, not face radius.
        if repass_pitches > 0:
            A_r_face = (D - d_path) / 2 + repass_offset / 2
            if abs(dia_end_offset) > 1e-9:
                A_r = A_r_face + dia_end_offset / 2
            else:
                A_r = A_r_face
            rp_z = -(L + P8) if is_bu else -(L - repass_pitches * lead) + P8
            continuous_helix(A_r, repass_pitches, rp_z, dia_taper=0.0)
        # MULTI-START: replicate the single-start cutting, each rotated by
        # k*360/num_starts about the workpiece axis.  Rotation is baked into
        # REAL X/Y/I/J coordinates (NOT G68) so it runs on any control.
        # lead = num_starts*P already advances Z so each helix covers depth;
        # end-points inherit the single-start plunge_z math.
        if num_starts > 1:
            _cut_body = list(lines[_cut_idx:])
            del lines[_cut_idx:]
            _step = 360.0 / num_starts

            def _rot_xy(x, y, ct, stt):
                # rotate a point OR an incremental vector about the axis (CCW)
                nx = x * ct - y * stt
                ny = x * stt + y * ct
                # snap floating-point dust (e.g. sin 180 = 1.2e-16) to 0
                if abs(nx) < 1e-9: nx = 0.0
                if abs(ny) < 1e-9: ny = 0.0
                return (nx, ny)

            def _rot_line(ln, ct, stt):
                # rotate the X&Y pair and the I&J arc-centre pair in one G-code
                # line.  Z, K, R, F, S, D, H and all G/M codes are untouched.
                if ln.lstrip().startswith('('):
                    return ln
                toks = ln.split(' ')
                pos = {}
                for ix, tk in enumerate(toks):
                    if len(tk) >= 2 and tk[0] in 'XYIJ' and (tk[1].isdigit() or tk[1] in '+-.'):
                        try:
                            pos[tk[0]] = (ix, float(tk[1:]))
                        except ValueError:
                            pass
                if 'X' in pos and 'Y' in pos:
                    nx, ny = _rot_xy(pos['X'][1], pos['Y'][1], ct, stt)
                    toks[pos['X'][0]] = 'X' + fmt(nx)
                    toks[pos['Y'][0]] = 'Y' + fmt(ny)
                if 'I' in pos and 'J' in pos:
                    ni, nj = _rot_xy(pos['I'][1], pos['J'][1], ct, stt)
                    toks[pos['I'][0]] = 'I' + fmt(ni)
                    toks[pos['J'][0]] = 'J' + fmt(nj)
                return ' '.join(toks)

            for _k in range(num_starts):
                _ang = (entry_angle + _k * _step) % 360.0
                lines.append(f'( ===== START {_k+1} OF {num_starts}  AT {fmt(_ang)} DEG ===== )')
                if _k > 0:
                    lines.append(f'G90 G00 Z{SAFE_Z}.')
                if abs(_ang) < 1e-9:
                    # identity (start 0 with no entry angle) -> verbatim cutting
                    if _k > 0:
                        lines.append(f'G90 G00 X{fmt(_start_xy[0])} Y{fmt(_start_xy[1])}')
                    lines.extend(_cut_body)
                else:
                    _ct = math.cos(math.radians(_ang))
                    _stt = math.sin(math.radians(_ang))
                    _rx, _ry = _rot_xy(_start_xy[0], _start_xy[1], _ct, _stt)
                    lines.append(f'G90 G00 X{fmt(_rx)} Y{fmt(_ry)}')
                    for _ln in _cut_body:
                        lines.append(_rot_line(_ln, _ct, _stt))

    elif mode == 'Stepped':
        # U152: Pitch-aligned axial passes for stepped milling.
        #
        # Bug history: U137/U139 used a uniform N*P inter-pass step and
        # overrode only the LAST pass's plunge_z to land at -L.  That
        # made the last pass land correctly, but adjacent passes ended
        # up shifted by NON-integer multiples of P → pass i+1's teeth
        # cut the thread at axial positions that didn't match pass i's
        # teeth → operator saw doubled / overlapping thread crests in
        # the overlap zone.
        #
        # New rule (per user, 2026-04-29 with example program for
        # M22x1.5, L=18.5, N=9):
        #   1. Inter-pass shift k must be an INTEGER number of pitches.
        #      That makes pass i+1's teeth re-trace pass i's teeth in
        #      the overlap region (no off-pitch crests).
        #   2. Last pass cutter_bottom orbit end = -L exactly (TTB),
        #      or pass 0 cutter_bottom orbit start = -L exactly (BTT).
        #   3. First pass top tooth orbit start lands in (-P, 0] for
        #      TTB.  Operator considers up-to-1-pitch face slop OK.
        #
        # Math: with n axial passes and uniform shift k pitches,
        #   TTB:  T (top tooth at orbit start of pass 0)
        #         = -L + N*P + (n-1)*k*P
        #         k = floor((L - N*P) / ((n-1)*P)) → T lands in (-P, 0].
        #         k clipped to [1, N] (k=0 = no advance; k>N leaves
        #         uncut gap between passes).
        #
        # Worked example (M22x1.5, L=18.5, P=1.5, N=9, TTB):
        #   n=2, k = floor((18.5-13.5)/1.5) = floor(3.333) = 3.
        #   T = -18.5 + 13.5 + 1*3*1.5 = -0.5 (top tooth 0.5 mm below face).
        #   Pass 0 plunge = T - 0 - (N-1)*P + P/8
        #                 = -0.5 - 12 + 0.1875 = -12.3125.
        #   Pass 1 plunge = T - k*P - (N-1)*P + P/8
        #                 = -0.5 - 4.5 - 12 + 0.1875 = -16.8125.
        #   Pass 1 cutter_b at orbit end
        #                 = -16.8125 - P/8 - lead = -18.5 = -L ✓
        #   Inter-orbit shift = 3*P = 4.5 mm (integer pitch → aligned).
        #
        # U124: lead = num_starts * P.  Used for orbit Z advance.
        # U152/U153: coverage_per_pass = (N-1)*P + lead.  For single-start
        # this reduces to N*P (same as old code).  For multi-start
        # (lead > P), each orbit advances Z by `lead`, so a single
        # orbit covers (N-1)*P (cutter span) + lead (Z advance) axially.
        #
        # U153: stepped_style decides the inter-pass strategy:
        #
        #   'Modified' (default) - TOP-of-thread first ALWAYS (TTB and BTT),
        #     inter-pass shift = integer*P (pitch-aligned), pass 1 fully
        #     engaged in workpiece (no air cutting).  k = floor((L -
        #     coverage)/((n-1)*P)) → top tooth at orbit start of pass 0
        #     lands in (-P, 0].
        #
        #   'Carmex' - matches Carmex Tool Wizard output: cutter follows
        #     its NATURAL motion direction across passes (TTB top→bottom;
        #     BTT bottom→top), inter-pass shift = N*P (= cutter coverage,
        #     contiguous tiling), last pass at -L exactly.  Pass 1 may
        #     have top tooth above face for short L (cutter teeth in air)
        #     but produces the SAME thread crests as Modified — just a
        #     different traversal order.
        N = num_teeth
        coverage = (N - 1) * P + lead
        # U156: sub-micron-perfect TTB last-pass plunge_z.  Compensate
        # for emitted-Z rounding so cutter_b at orbit_end lands at
        # exactly -L.  For tapered: 4 quads × fmt(zs*lead/4); for
        # straight: 1 full circle × fmt(zs*lead).  When lead/4 doesn't
        # round cleanly to 4 decimals (e.g. NPT lead=1.8143 → lead/4 =
        # 0.453575 → fmt 0.4536), the per-orbit emission is +0.0001
        # off theoretical.  Without compensation, last pass orbit_end
        # is also off by 0.0001 per orbit.
        is_tapered_step = abs(bottom_offset) > 1e-9
        entry_emit_TTB = float(fmt(-P8))
        if is_tapered_step:
            orbit_emit_TTB = 4 * float(fmt(-lead / 4.0))
        else:
            orbit_emit_TTB = float(fmt(-lead))
        # plunge_z + entry_emit + orbit_emit = -L  → solve for plunge_z
        last_plunge_TTB_compensated = -L - entry_emit_TTB - orbit_emit_TTB
        if L <= coverage:
            # Single axial pass suffices.  Bottom_end (TTB) or bottom_start
            # (BTT) lands exactly at -L.  For L < coverage, some teeth sit
            # above the work face — that's the operator's choice of cutter.
            if is_bu:
                # BTT: cutter_b at orbit start = -L (deepest point).
                # plunge_z = -L - P/8 (entry ascends by P/8 — fmt(P/8)
                # is exact for typical P, so no compensation needed).
                plunge_zs = [-L - P8]
            else:
                # TTB: cutter_b at orbit end = -L (sub-micron-compensated).
                plunge_zs = [last_plunge_TTB_compensated]
        elif 'Carmex' in stepped_style:
            # Carmex strategy: walk by full cutter coverage (N*P) per pass,
            # last pass lands at -L exactly.  Pass order follows cutter's
            # natural motion direction.
            step = N * P
            n_passes_step = max(2, math.ceil(L / step))
            if is_bu:
                # BTT Carmex: pass 0 = DEEPEST (cutter starts at bottom,
                # ascends through pass 0, then ascends to start pass 1
                # higher up).  plunge_z_0 = -L - P/8 (cutter_b lands at
                # -L after entry ascends P/8).  Subsequent passes shift
                # UP by step = N*P.
                plunge_zs = [-L - P8 + i * step
                             for i in range(n_passes_step)]
            else:
                # TTB Carmex: pass 0 = HIGHEST (top tooth often above
                # face).  plunge_z_(n-1) = sub-micron-compensated (lands
                # cutter_b at orbit_end = -L exactly).  Pass i is +step
                # ABOVE pass i+1.
                last_plunge = last_plunge_TTB_compensated
                plunge_zs = [last_plunge + (n_passes_step - 1 - i) * step
                             for i in range(n_passes_step)]
        else:
            # 'Modified' (default).  Multi-pass with pitch-aligned shift.
            #
            # U162 / U162B (locked 2026-05-04 after walkthrough with user
            # on M30×2.5, various L values for both TTB and BTT):
            #
            #   TTB pass 0 top tooth orbit-START lands in [0, +P) —
            #     at face (clean fit) or up to just under +P (rounding).
            #     NEVER below face.  Top tooth descends THROUGH face
            #     during orbit, forming the topmost crest.
            #
            #   BTT pass 0 top tooth orbit-START lands in (-2P, -P] —
            #     at -P (clean fit) or up to just above -2P (rounding).
            #     NEVER above -P.  Top tooth ascends from below face
            #     through (or up to near) face, forming the topmost
            #     crest.
            #
            # Both directions assume the operator has pre-machined a
            # chamfer (≥ 1 pitch deep) at the bore mouth.  That plus
            # the existing P/8 entry quarter-arc gives the top tooth a
            # smooth geometric lead-in: cutter transitions from chamfer
            # air space → face → engaged material gradually, no
            # abrupt-engagement burr or chipping.
            #
            # Math summary (single-start; coverage = (N-1)*P + lead):
            #   TTB target: (L - coverage)/P, rounded UP (ceil).
            #   BTT target: (L - N*P)/P,      rounded DOWN (floor).
            #   Both reduce to (L/P - N) for single-start.
            #
            # Method (mirrored for both directions):
            #   1. total = ceil/floor of target; eps so exact ints
            #      don't shift by 1.
            #   2. T_0 from total via inverse formula.
            #   3. n = ceil(total/N) + 1 (smallest pass count where
            #      (n-1) hops, each ≤ N, can sum to `total`).
            #   4. Build shift array greedily, BIGGER FIRST.
            #   5. plunge_zs derived from T values per direction:
            #      TTB: plunge_z = T - (N-1)*P + P/8
            #      BTT: plunge_z = T - (N-1)*P - P/8
            #   6. TTB last pass: U156 sub-micron correction.
            #
            # Worked examples (P=2.5, single-start, see code below):
            #   TTB L=20  N=3: shifts=[3,2],   T_0=0,    n=3
            #   TTB L=23  N=3: shifts=[3,3,1], T_0=+2,   n=4
            #   TTB L=20  N=5: shifts=[3],     T_0=0,    n=2
            #   BTT L=20  N=3: shifts=[3,2],   T_0=-P,   n=3
            #   BTT L=23  N=3: shifts=[3,3],   T_0=-1.2P,n=3
            #   BTT L=20  N=5: shifts=[3],     T_0=-P,   n=2
            #
            # Single-pass case (L ≤ coverage) is handled by the earlier
            # "if L <= coverage" branch — no change needed there.
            if is_bu:
                # BTT — U162B floor-based with variable-k bigger-first
                # shifts.  Mirror of TTB rule but on the bottom side.
                #
                # RULE (locked 2026-05-04): Pass 0 top tooth orbit-START
                # lands at -P (when math fits cleanly) or up to JUST
                # above -2P below face (when fractional pitches force
                # rounding down).  NEVER above -P.  After orbit (BTT
                # ascends by `lead`), top tooth ends at face (single-
                # start, T_0=-P) or above face (chamfer space).
                #
                # The chamfer at the bore mouth absorbs the engagement
                # transition just like TTB.  When fractional L forces
                # T_0 < -P, the topmost full crest forms 1 pitch below
                # face (= within or just below the chamfer's transition
                # zone) — chamfer geometry covers the cosmetic gap.
                #
                # Math:
                #   T_0 = top tooth orbit-START of pass 0
                #       = -L + (N-1)*P + (n-1)*k_total*P
                #   For T_0 ≤ -P: (n-1)*k_total ≤ (L - N*P)/P
                #   →  use FLOOR search with this target.
                #
                # Worked examples (P=2.5, single-start):
                #   L=20,   N=3:  total=5, n=3, shifts=[3,2],   T_0=-P
                #                 plunge_zs = [-7.8125, -15.3125, -20.3125]
                #   L=20.5, N=3:  total=5, n=3, shifts=[3,2],   T_0=-1.2P
                #                 plunge_zs = [-8.3125, -15.8125, -20.8125]
                #   L=23,   N=3:  total=6, n=3, shifts=[3,3],   T_0=-1.2P
                #                 plunge_zs = [-8.3125, -15.8125, -23.3125]
                #   L=20,   N=5:  total=3, n=2, shifts=[3],     T_0=-P
                #                 plunge_zs = [-12.8125, -20.3125]
                #   L=25.1, N=5:  total=5, n=2, shifts=[5],     T_0=-1.04P
                #                 plunge_zs = [-12.9125, -25.4125]
                #
                # Single-pass (L ≤ coverage) handled above; T_0 lands at
                # -L + (N-1)*P naturally — may be > -P for short L, but
                # that's geometrically forced (can't go more below
                # without overcutting past -L).
                target = (L - N * P) / P  # = L/P - N for single-start
                total_pitches = max(0, int(math.floor(target + 1e-9)))
                if total_pitches == 0:
                    # Edge case: L just slightly > coverage.  Multi-pass
                    # math gives total=0 (= single-pass equivalent).
                    # Fall back to single-pass plunge_z.  T_0 will be
                    # at -L + (N-1)*P which is ≤ -P here since
                    # L > coverage = N*P (single-start).
                    n_passes_step = 1
                    plunge_zs = [-L - P8]
                else:
                    # Smallest n: (n-1) hops × N ≥ total_pitches.
                    n_passes_step = int(math.ceil(total_pitches / float(N))) + 1
                    # Build shift array (length n-1), bigger first.
                    shifts_list = []
                    remaining = total_pitches
                    hops_total = n_passes_step - 1
                    for hop_idx in range(hops_total):
                        future_hops = hops_total - hop_idx - 1
                        lo = max(1, remaining - future_hops * N)
                        hi = min(N, remaining)
                        hop = hi if hi >= lo else lo
                        shifts_list.append(hop)
                        remaining -= hop
                    # Build T values (top tooth orbit-START of each pass).
                    T_0 = -L + (N - 1) * P + total_pitches * P
                    T_values = [T_0]
                    T_current = T_0
                    for shift in shifts_list:
                        T_current -= shift * P
                        T_values.append(T_current)
                    # plunge_z = bot tooth orbit-START - P/8.
                    # (BTT entry ASCENDS P/8: cutter_b before entry is
                    # P/8 BELOW orbit-START position.)
                    # bot tooth orbit-START = T - (N-1)*P.
                    plunge_zs = [T - (N - 1) * P - P8 for T in T_values]
            else:
                # TTB — U162 ceil-based with variable-k bigger-first
                # shifts.
                target = (L - coverage) / P
                # eps so an exact integer doesn't get bumped up by 1.
                total_pitches = max(0, int(math.ceil(target - 1e-9)))
                if total_pitches == 0:
                    # Defensive: shouldn't happen here (single-pass
                    # branch above handles L <= coverage), but if a
                    # pathological rounding case sneaks in, treat as
                    # 2 passes with one minimum shift.
                    n_passes_step = 2
                    shifts_list = [1]
                    total_pitches = 1
                else:
                    # Smallest n: (n-1) hops × N ≥ total_pitches
                    n_passes_step = int(math.ceil(total_pitches / float(N))) + 1
                    # Build shift array (length n-1), bigger first.
                    shifts_list = []
                    remaining = total_pitches
                    hops_total = n_passes_step - 1
                    for hop_idx in range(hops_total):
                        future_hops = hops_total - hop_idx - 1
                        # This hop must leave at most future_hops*N for
                        # the remaining hops, and each future hop ≥ 1.
                        lo = max(1, remaining - future_hops * N)
                        hi = min(N, remaining)
                        # Bigger-first: take the maximum allowed.
                        hop = hi if hi >= lo else lo
                        shifts_list.append(hop)
                        remaining -= hop
                # Build T values (top tooth orbit-START of each pass).
                T_0 = -L + coverage + total_pitches * P
                T_values = [T_0]
                T_current = T_0
                for shift in shifts_list:
                    T_current -= shift * P
                    T_values.append(T_current)
                # plunge_z = cutter_b at orbit-START + P/8 (entry
                # descent) = (T - (N-1)*P) + P/8.
                plunge_zs = [T - (N - 1) * P + P8 for T in T_values]
                # U156: sub-micron-perfect last pass — override the
                # last plunge_z with the rounding-compensated value so
                # cutter_b at orbit_end lands at exactly -L.  Other
                # passes keep their pitch-aligned positions; the shift
                # on the last pass is sub-micron so alignment isn't
                # visibly broken.
                plunge_zs[-1] = last_plunge_TTB_compensated
        # U153: top_first = "is pass 0 at the TOP of the thread?"
        # Modified: True always (top-first rule).
        # Carmex:   True for TTB, False for BTT (cutter natural motion).
        # Used by stepped_sequence to assign correct per-pass radius
        # for tapered threads (pass at top → smallest A_base offset).
        if 'Carmex' in stepped_style:
            top_first = not is_bu
        else:
            top_first = True
        for p_idx in range(num_passes):
            pct = pcts[p_idx]
            if pct <= 0:
                continue
            # U46 (internal) / U157 (external): A_base depends on side.
            if is_external:
                A_base = (D + d_path) / 2 - (pct / 100) * thread_depth
            else:
                A_base = (D - d_path) / 2 - (1 - pct / 100) * thread_depth
            # U158: bump spindle to Finishing RPM before last pass.
            if is_finishing and p_idx == last_radial_idx:
                lines.append(f'S{finishing_rpm}')
            stepped_sequence(A_base, plunge_zs, taper=bottom_offset,
                             feed_multiplier=feed_mults[p_idx],
                             top_first=top_first)
        # U6: stepped-mode bottom repass also runs when offset=0
        # U124: repass last-pass plunge accounts for lead.
        if repass_pitches > 0:
            A_r = (D - d_path) / 2 + repass_offset / 2
            bottom_plunge = -L - P8 if is_bu else -L + lead + P8
            stepped_sequence(A_r, [bottom_plunge], taper=0,
                             top_first=top_first)

    lines.append(f'G90 G00 Z{SAFE_Z}.')
    if use_g68 and num_starts == 1:
        # U159: cancel coordinate-system rotation before M30.
        lines.append('G69')
    lines.append('M30')
    lines.append('%')
    # U135: no extra rapid time at end - the single RAPID_TIME budget at
    # program start covers all G00 + initial plunge + final retract.
    return '\n'.join(lines), st['time']


def generate(D, L, P, d, Z, Vc, fz, hand='RH', direction='Bottom to Top',
             mode='Helical', num_passes=1, pcts=(100, 0, 0, 0),
             repass_pitches=0, repass_offset=0.0,
             num_teeth=3, bottom_offset=0.0, spindle='M03',
             dia_end_offset=0.0, minor=None, entry_style='45', num_starts=1,
             entry_feed_reduction=50.0, max_rpm=5000,
             stepped_style='Modified', is_external=False,
             strategy='Standard', entry_angle=0.0,
             tool_offset_mode='centre', rctf_strength=0.0):
    program, _ = _build(D, L, P, d, Z, Vc, fz, hand, direction, mode,
                        num_passes, pcts, repass_pitches, repass_offset,
                        num_teeth, bottom_offset, spindle,
                        dia_end_offset=dia_end_offset, minor=minor,
                        entry_style=entry_style, num_starts=num_starts,
                        entry_feed_reduction=entry_feed_reduction,
                        max_rpm=max_rpm, stepped_style=stepped_style,
                        is_external=is_external, strategy=strategy,
                        entry_angle=entry_angle,
                        tool_offset_mode=tool_offset_mode,
                        rctf_strength=rctf_strength)
    return program


def cycle_time(D, L, P, d, Z, Vc, fz, hand='RH', direction='Bottom to Top',
               mode='Helical', num_passes=1, pcts=(100, 0, 0, 0),
               repass_pitches=0, repass_offset=0.0,
               num_teeth=3, bottom_offset=0.0, spindle='M03',
               dia_end_offset=0.0, minor=None, entry_style='45', num_starts=1,
               entry_feed_reduction=50.0, max_rpm=5000,
               stepped_style='Modified', is_external=False,
               strategy='Standard', entry_angle=0.0,
               tool_offset_mode='centre', rctf_strength=0.0):
    _, t = _build(D, L, P, d, Z, Vc, fz, hand, direction, mode,
                  num_passes, pcts, repass_pitches, repass_offset,
                  num_teeth, bottom_offset, spindle,
                  dia_end_offset=dia_end_offset, minor=minor,
                  entry_style=entry_style, num_starts=num_starts,
                  entry_feed_reduction=entry_feed_reduction,
                  max_rpm=max_rpm, stepped_style=stepped_style,
                  is_external=is_external, strategy=strategy,
                  entry_angle=entry_angle,
                  tool_offset_mode=tool_offset_mode,
                  rctf_strength=rctf_strength)
    return t
