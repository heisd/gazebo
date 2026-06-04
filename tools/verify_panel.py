#!/usr/bin/env python3
"""Closed-loop verification for the scheduler bug fixes + web control panel.

Runs the *real* ``AGVScheduler`` through the in-process simulation backend used
by ``tools/control_panel.py`` and asserts, end to end:

  * the operator command channel (goto / stop / return / charge / task) moves
    the two AGVs as instructed;
  * each of the six reviewed bugs is actually fixed.

Exit code is non-zero if any check fails.  No ROS / Gazebo required.

    python3 tools/verify_panel.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

from control_panel import SimBackend  # noqa: E402

RESULTS = []


def check(name, ok, detail=""):
    RESULTS.append((name, bool(ok), detail))
    mark = "PASS" if ok else "FAIL"
    line = f"  [{mark}] {name}"
    if detail:
        line += f"  — {detail}"
    print(line)


def fresh():
    """A backend with the two AGVs parked at their home wait points."""
    return SimBackend(rate=1.0)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


# ==========================================================================
# Panel control: the web buttons must actually drive the fleet
# ==========================================================================
def test_manual_goto():
    b = fresh()
    State = b.mod.State
    b.command({"cmd": "goto", "agv_id": "agv_01", "x": -5.0, "y": 1.0})
    b.step_for(80)
    f = b.status()["fleet"]["agv_01"]
    arrived = dist(f["pos"], [-5.0, 1.0]) < 0.5
    check("manual goto drives AGV to target and parks",
          arrived and f["state"] == State.IDLE.value and f["task"] is None,
          f"pos={f['pos']} state={f['state']}")


def test_manual_stop():
    b = fresh()
    State = b.mod.State
    b.command({"cmd": "goto", "agv_id": "agv_02", "x": 0.0, "y": 0.0})
    b.step_for(3)
    moving_mid = b.sched.agvs["agv_02"].current_goal_handle is not None
    b.command({"cmd": "stop", "agv_id": "agv_02"})
    b.step_for(1)
    agv = b.sched.agvs["agv_02"]
    check("manual stop cancels the goal and idles the AGV",
          moving_mid and agv.state == State.IDLE
          and agv.task is None and agv.current_goal_handle is None,
          f"state={agv.state.value}")


def test_manual_return_and_charge():
    b = fresh()
    State = b.mod.State
    # drive away first, then send home
    b.command({"cmd": "goto", "agv_id": "agv_01", "x": -1.0, "y": -1.0})
    b.step_for(40)
    b.command({"cmd": "return", "agv_id": "agv_01"})
    b.step_for(120)
    f1 = b.status()["fleet"]["agv_01"]
    home = dist(f1["pos"], [10.8, 4.0]) < 1.0
    check("manual return sends the AGV home and idles",
          home and f1["state"] == State.IDLE.value, f"pos={f1['pos']}")

    # start part-charged so the AGV stays in CHARGING (a full battery would
    # immediately cycle back to idle once docked)
    b.sched.agvs["agv_02"].battery = 50.0
    b.command({"cmd": "charge", "agv_id": "agv_02"})
    b.step_for(40)
    a2 = b.sched.agvs["agv_02"]
    at_charger = dist((a2.x, a2.y), b.sched.charging_xy) < 1.0
    check("manual charge reaches the charger and starts charging",
          a2.state == State.CHARGING and at_charger,
          f"state={a2.state.value} pos=({a2.x:.1f},{a2.y:.1f})")


def test_end_to_end_fleet():
    b = fresh()
    for tid, shelf in [("J1", "A1"), ("J2", "D4"), ("J3", "B2"), ("J4", "C3")]:
        b.command({"cmd": "task", "shelf": shelf, "priority": 5,
                   "agv_id": None, "tid": tid})
    b.step_for(240)
    st = b.status()
    check("closed-loop: 4 dispatched tasks all complete",
          st["completed"] >= 4, f"completed={st['completed']}")


# ==========================================================================
# Bug fixes
# ==========================================================================
def test_fix1_emergency_preempts_reservation():
    """Emergency charge must not be stranded by another AGV's reservation."""
    b = fresh()
    State = b.mod.State
    Task = b.mod.Task
    s = b.sched
    a1, a2 = s.agvs["agv_01"], s.agvs["agv_02"]
    a1.x, a1.y = 8.0, -6.0
    a2.x, a2.y = 8.4, -6.2

    def charge_task(aid):
        c = s.charging_xy
        return Task(tid=f"CHARGE_{aid}", shelf="", shelf_center_xy=c,
                    pick_xy=c, pick_yaw=0.0, aisle_exit_xy=c, drop_xy=c,
                    priority=0, agv=aid, status="charging", kind="charge")

    with s.lock:
        # agv_02 grabs the charger route first
        blk2, _ = s._reserve_stage_locked(
            a2, charge_task("agv_02"), State.TO_CHARGE, start_xy=(a2.x, a2.y))
        # old behaviour: agv_01 would be blocked
        blk_noprempt, _ = s._reserve_stage_locked(
            a1, charge_task("agv_01"), State.TO_CHARGE,
            start_xy=(a1.x, a1.y), preempt=False)
        # fix: with preempt the emergency mover takes the route
        blk_preempt, _ = s._reserve_stage_locked(
            a1, charge_task("agv_01"), State.TO_CHARGE,
            start_xy=(a1.x, a1.y), preempt=True)
        a1_owns = any(r.agv_id == "agv_01"
                      for r in s.route_reservations.values())
    check("fix#1 emergency charge preempts a blocking reservation",
          blk2 == "" and blk_noprempt == "agv_02"
          and blk_preempt == "" and a1_owns,
          f"noPreempt_blocker={blk_noprempt!r} preempt_blocker={blk_preempt!r}")


def test_fix2_waitpoint_retreat_vs_hold():
    """Real task stages retreat to a wait point; internal moves hold in place."""
    b = fresh()
    State = b.mod.State
    Task = b.mod.Task
    s = b.sched
    a1, a2 = s.agvs["agv_01"], s.agvs["agv_02"]

    def shelf_task(tid):
        loc = s.shelves["B2"]
        return Task(tid=tid, shelf="B2", shelf_center_xy=loc.center_xy,
                    pick_xy=loc.pick_xy, pick_yaw=loc.pick_yaw,
                    aisle_exit_xy=(5.5, loc.pick_xy[1]), drop_xy=s.station_xy,
                    priority=5, kind="shelf")

    with s.lock:
        # agv_02 reserves the station leg -> grabs the exclusive dock:station
        s._reserve_stage_locked(a2, shelf_task("BLOCKER"), State.TO_STATION,
                                start_xy=(a2.x, a2.y))
        prep = s._prepare_stage_dispatch_locked(
            a1, shelf_task("T_REAL"), State.TO_STATION, wait_on_block=True)

    real_task = shelf_task("T_REAL")
    internal_task = Task(tid="RETURN_agv_01", shelf="", shelf_center_xy=(0, 0),
                         pick_xy=(0, 0), pick_yaw=0.0, aisle_exit_xy=(0, 0),
                         drop_xy=(0, 0), priority=0, kind="return")

    retreats = (prep["action"] == "wait" and prep.get("hold_position") is False
                and prep.get("wait_point") is not None
                and s._wait_should_hold_position(real_task, prep) is False)
    holds = s._wait_should_hold_position(internal_task,
                                         {"hold_position": True}) is True
    # internal task holds even when a wait point exists (hold_position False)
    holds2 = s._wait_should_hold_position(internal_task,
                                          {"hold_position": False}) is True
    check("fix#2 blocked real stage retreats to a wait point (not hold)",
          retreats, f"action={prep['action']} hold={prep.get('hold_position')} "
                    f"wp={prep.get('wait_point') is not None}")
    check("fix#2/#3 internal moves still hold in place; wait-point path live",
          holds and holds2)


def test_fix4_reserved_prefix_rejected():
    b = fresh()
    b.step_for(1)  # let the status feed publish once
    before = b.status()["pending"]
    # external task masquerading as an internal move must be ignored
    b.command({"cmd": "task", "shelf": "A1", "priority": 5,
               "tid": "RETURN_evil"})
    b.step_for(0.5)
    after = b.status()["pending"]
    logs = " ".join(x["text"] for x in b.status()["log"])
    check("fix#4 external task with reserved prefix is rejected",
          after == before and "reserved internal prefix" in logs,
          f"pending {before}->{after}")


def test_fix4_kind_discrimination():
    b = fresh()
    Task = b.mod.Task
    s = b.sched
    # a genuine shelf task whose tid is mundane still classified as shelf
    t = Task(tid="T9", shelf="A1", shelf_center_xy=(0, 0), pick_xy=(0, 0),
             pick_yaw=0.0, aisle_exit_xy=(0, 0), drop_xy=(0, 0), priority=1)
    charge = Task(tid="CHARGE_agv_01", shelf="", shelf_center_xy=(0, 0),
                  pick_xy=(0, 0), pick_yaw=0.0, aisle_exit_xy=(0, 0),
                  drop_xy=(0, 0), priority=0, kind="charge")
    check("fix#4 internal/external split is by kind, not tid prefix",
          s._internal_stage_for_task(t) is None
          and s._internal_stage_for_task(charge) is not None)


def test_fix5_completed_counter_and_bounded_history():
    from collections import deque
    b = fresh()
    s = b.sched
    check("fix#5 history is a bounded ring (no unbounded growth)",
          isinstance(s.history, deque) and s.history.maxlen == s.HISTORY_LIMIT,
          f"maxlen={s.history.maxlen}")

    # complete some real work, then flood history; completed must not regress
    for tid, shelf in [("H1", "A1"), ("H2", "D4")]:
        b.command({"cmd": "task", "shelf": shelf, "priority": 5, "tid": tid})
    b.step_for(160)
    completed_after_work = b.status()["completed"]
    Task = b.mod.Task
    with s.lock:
        for i in range(s.HISTORY_LIMIT + 80):
            s.history.append(Task(tid=f"X{i}", shelf="", shelf_center_xy=(0, 0),
                                  pick_xy=(0, 0), pick_yaw=0.0,
                                  aisle_exit_xy=(0, 0), drop_xy=(0, 0)))
    check("fix#5 completed count survives history trimming",
          completed_after_work >= 2
          and b.status()["completed"] == completed_after_work
          and len(s.history) == s.HISTORY_LIMIT,
          f"completed={completed_after_work} hist_len={len(s.history)}")


def test_fix6_internal_retry_backoff_rearm():
    b = fresh()
    State = b.mod.State
    Task = b.mod.Task
    s = b.sched
    a1, a2 = s.agvs["agv_01"], s.agvs["agv_02"]
    a1.x, a1.y = 8.0, -6.0
    a2.x, a2.y = 8.4, -6.2
    b.sim.SimClock.t = 1000.0
    charge1 = Task(tid="CHARGE_agv_01", shelf="", shelf_center_xy=s.charging_xy,
                   pick_xy=s.charging_xy, pick_yaw=0.0,
                   aisle_exit_xy=s.charging_xy, drop_xy=s.charging_xy,
                   priority=0, agv="agv_01", status="charging", kind="charge")

    def charge2():
        c = s.charging_xy
        return Task(tid="CHARGE_agv_02", shelf="", shelf_center_xy=c,
                    pick_xy=c, pick_yaw=0.0, aisle_exit_xy=c, drop_xy=c,
                    priority=0, agv="agv_02", status="charging", kind="charge")

    with s.lock:
        # block the charger route with agv_02 so the retry stays blocked
        s._reserve_stage_locked(a2, charge2(), State.TO_CHARGE,
                                start_xy=(a2.x, a2.y))
        s._set_internal_retry_wait_locked(a1, charge1, "nav2 not ready",
                                           retry_delay=5.0)
    first_deadline = a1.wait_until                 # 1005.0
    # jump past the first deadline so the retry fires, but it is still blocked
    b.sim.SimClock.t = first_deadline + 0.1        # 1005.1
    s._right_of_way_loop()
    rearmed = a1.wait_until > b.sim.SimClock.t      # pushed back into the future
    check("fix#6 still-blocked internal retry re-arms its backoff (no spin)",
          a1.state == State.WAITING and rearmed
          and a1.wait_until >= first_deadline + 5.0,
          f"deadline {first_deadline}->{a1.wait_until} now={b.sim.SimClock.t}")


def main():
    print("=" * 70)
    print("AGV control-panel + bug-fix closed-loop verification")
    print("=" * 70)
    print("\n-- Panel control (operator commands drive the fleet) --")
    test_manual_goto()
    test_manual_stop()
    test_manual_return_and_charge()
    test_end_to_end_fleet()
    print("\n-- Reviewed bug fixes --")
    test_fix1_emergency_preempts_reservation()
    test_fix2_waitpoint_retreat_vs_hold()
    test_fix4_reserved_prefix_rejected()
    test_fix4_kind_discrimination()
    test_fix5_completed_counter_and_bounded_history()
    test_fix6_internal_retry_backoff_rearm()

    passed = sum(1 for _, ok, _ in RESULTS if ok)
    total = len(RESULTS)
    print("\n" + "=" * 70)
    print(f"RESULT: {passed}/{total} checks passed")
    print("=" * 70)
    return 0 if passed == total else 1


if __name__ == "__main__":
    sys.exit(main())
