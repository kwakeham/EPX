"""Offline unit tests for the console grammar (no hardware). Run: ``pytest``."""

from epx_hil import protocol
from epx_hil.models import LineTag


def test_command_builders_exact_forms():
    # Fixed-index handlers take NO space (ma/mg/gl/gh/gi); atoi/sscanf ones may.
    assert protocol.cmd_mode_angle() == "ma"
    assert protocol.cmd_mode_gear() == "mg"
    assert protocol.cmd_capture_low() == "gl"
    assert protocol.cmd_capture_high() == "gh"
    assert protocol.cmd_interpolate() == "gi"
    assert protocol.cmd_shift_to_index(0) == "s0"     # logical gear 1
    assert protocol.cmd_shift_to_index(9) == "s9"     # logical gear 10
    assert protocol.cmd_shift_to_index(10) == "s10"   # logical gear 11
    assert protocol.cmd_set_angle(-89.0) == "t-89.00"
    assert protocol.cmd_telemetry(2) == "y2"
    assert protocol.cmd_events(63) == "e63"
    assert protocol.cmd_isense_limit(500) == "xl 500"
    assert protocol.cmd_reboot() == "r"


def test_classify_line():
    assert protocol.classify_line(protocol.CSV_HEADER) is LineTag.CSV_HEADER
    assert protocol.classify_line("0,35000,34950,50,120,250,1,500,0") is LineTag.CSV_ROW
    assert protocol.classify_line("123,-8900,-8901,1,-5,0,0,12,0") is LineTag.CSV_ROW
    assert protocol.classify_line("#boot,rot=5,gear=9,pos=-89,tgt=-89") is LineTag.EVENT
    assert protocol.classify_line("GEAR g9 pos -89 tgt -89 err 0 I 12 HLD") is LineTag.STATUS
    assert protocol.classify_line("EPX console ready. Type 'h'.") is LineTag.BANNER
    assert protocol.classify_line("Gains: Kp: 3.000, Ki: 0.000, Kd: 0.000") is LineTag.REPLY
    assert protocol.classify_line("Low ref (gear 2): 5303") is LineTag.REPLY


def test_parse_status_sample():
    st = protocol.parse_status("GEAR g9 pos -89 tgt -89 err 0 I 12 HLD")
    assert st is not None
    assert st.mode == "GEAR" and st.gear == 9 and st.logical_gear == 10
    assert st.pos_deg == -89 and st.tgt_deg == -89 and st.err_deg == 0
    assert st.isense == 12 and st.moving is False and st.fault is False


def test_parse_status_moving_fault():
    st = protocol.parse_status("ANGLE g2 pos 12000 tgt 15000 err 3000 I 2500 MOV FAULT")
    assert st.mode == "ANGLE" and st.moving is True and st.fault is True
    assert st.isense == 2500


def test_parse_csv_row():
    row = protocol.parse_csv_row("400,35000,34950,50,120,250,1,500,0")
    assert row == (400, 35000, 34950, 50, 120, 250, 1, 500, 0)
    assert protocol.parse_csv_row("bad,row") is None


def test_parse_events():
    ev = protocol.parse_event("#boot,rot=5,gear=9,pos=-89,tgt=-89", host_ts=1.0)
    assert ev.kind == "boot"
    assert ev.fields == {"rot": 5, "gear": 9, "pos": -89, "tgt": -89}
    assert ev.host_ts == 1.0
    sh = protocol.parse_event("#shift,gear=9,pos=-89,over=100")
    assert sh.kind == "shift" and sh.fields["over"] == 100
    cal = protocol.parse_event("#cal,step=lo,angle=53032")
    assert cal.kind == "cal" and cal.fields["step"] == "lo" and cal.fields["angle"] == 53032


def test_gear_table_assembly_and_mapping():
    lines = [
        "Gear 1: 6272, 5303, 4530, 3811, 3079, 2402",
        "Gear 7: 1731, 1137, 533, -89, -665, 0",
    ]
    tbl = protocol.gear_table_from_lines(lines)
    assert tbl is not None
    assert tbl.position_for(2) == 5303      # logical gear 2 -> index 1
    assert tbl.position_for(10) == -89      # logical gear 10 -> index 9
    assert tbl.position_for_index(0) == 6272
    assert tbl.is_calibrated() is True


def test_gear_table_uncalibrated_sentinel():
    zeros = protocol.gear_table_from_lines([
        "Gear 1: 0, 0, 0, 0, 0, 0", "Gear 7: 0, 0, 0, 0, 0, 0"])
    assert zeros.is_calibrated() is False


def test_parse_refs_and_gains():
    assert protocol.parse_low_ref("Low ref (gear 2): 5303") == 5303
    assert protocol.parse_high_ref("High ref (gear 10): -89") == -89
    assert protocol.parse_gains("Gains: Kp: 3.000, Ki: 0.500, Kd: 0.050") == (3.0, 0.5, 0.05)
