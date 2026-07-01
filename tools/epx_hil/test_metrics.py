"""Offline unit tests for metric math (no hardware). Run: ``pytest``."""

from epx_hil import metrics
from epx_hil.models import TelemetryRow


def row(t, tgt_cd, cur_cd, state, isense=100, fault=0):
    return TelemetryRow(t, tgt_cd, cur_cd, tgt_cd - cur_cd, 0, 0, state, isense, fault)


def test_overshoot_measured_against_final_not_waypoint():
    # target column steps through an overshift WAYPOINT (12000) before the final
    # gear position (10000). current peaks at 10300 (=103deg, 3deg past final).
    rows = [
        row(0, 0, 0, 0), row(8, 0, 0, 0),               # pre-move
        row(16, 12000, 0, 1),                           # t0: target leaves 0
        row(24, 12000, 3000, 1),
        row(32, 12000, 7000, 1),
        row(40, 12000, 9500, 1),                        # first in band (±10deg)
        row(48, 12000, 10300, 1),                       # peak: 3deg past final
        row(56, 10000, 10100, 1),                       # waypoint retired to final
        row(64, 10000, 10000, 1),
        row(72, 10000, 10000, 0),                       # MOV->HLD (settle)
        row(80, 10000, 10005, 0),
        row(88, 10000, 9998, 0),
    ]
    m = metrics.compute("t", rows, final_target_cd=10000, turn_events=0)
    assert m.start_deg == 0.0 and m.distance_deg == 100.0
    assert round(m.overshoot_deg, 2) == 3.0            # vs final 100, not waypoint 120
    assert round(m.overshoot_pct, 2) == 3.0
    assert m.time_to_target_ms == 24.0                  # t=40 - t0=16
    assert m.settle_time_fw_ms == 56.0                  # t=72 - t0=16
    assert m.steady_state_err_deg < 0.1
    assert m.turns_traversed == 0
    assert m.flags == []


def test_turns_traversed_and_event_crosscheck():
    rows = [
        row(0, 0, 0, 0),
        row(8, 40000, 0, 1), row(16, 40000, 20000, 1), row(24, 40000, 40000, 1),
        row(32, 40000, 40000, 0), row(40, 40000, 40000, 0),
    ]
    m = metrics.compute("t", rows, final_target_cd=40000, turn_events=1)
    assert m.turns_traversed == 1                       # floor(400/360) - floor(0/360)
    assert "turn_count_discrepancy" not in m.flags

    m2 = metrics.compute("t", rows, final_target_cd=40000, turn_events=2)
    assert "turn_count_discrepancy" in m2.flags         # counted 2 but only 1 crossing


def test_fault_and_settle_timeout_flags():
    rows = [
        row(0, 0, 0, 0),
        row(8, 10000, 2000, 1), row(16, 10000, 4000, 1, fault=1),  # fault mid-move
        row(24, 10000, 4000, 1),                                    # still MOV at end
    ]
    m = metrics.compute("t", rows, final_target_cd=10000, fault=True, settled=False)
    assert "fault" in m.flags
    assert "settle_timeout" in m.flags
    assert "never_in_band" in m.flags


def test_empty_window():
    m = metrics.compute("t", [], final_target_cd=10000)
    assert m.flags == ["no_rows"] and m.n_rows == 0
