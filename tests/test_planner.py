"""Planner behaviour, mostly against the real azula1 highway and trees."""

from __future__ import annotations

from dataclasses import replace

import pytest

from ct_nav import (
    Mode,
    RailPose,
    StepKind,
    Visit,
    arm_chain,
    stays_at_node,
    highway_route,
    plan_move,
    plan_park,
)
from ct_nav.planner import MAX_RAIL_MOVE_MM, PlanError


def rail_steps(plan):
    return [s for s in plan.steps if s.kind in (StepKind.HIGHWAY, StepKind.RAIL)]


def arm_steps(plan):
    return [s for s in plan.steps if s.kind is StepKind.ARM]


def last_rail(plan):
    """Final commanded rail pose, wherever it was set (JUMP folds it into one step)."""
    return next(s.rail for s in reversed(plan.steps) if not s.rail.is_empty())


# ---------------------------------------------------------------------------
# Highway routing
# ---------------------------------------------------------------------------

def test_route_to_self_is_a_single_node(cluster):
    arm = cluster.arm("mhr_xz")
    assert highway_route(arm, "iom_0", "iom_0") == ["iom_0"]


def test_route_climbs_to_the_common_ancestor_and_back_down(cluster):
    arm = cluster.arm("mhr_xz")
    # ncm_1 < ncm_0 < ncm_entry < iom_0, and icm_grex1_0 < icm_grex1_entry < iom_0.
    assert highway_route(arm, "ncm_1", "icm_grex1_0") == [
        "ncm_1",
        "ncm_0",
        "ncm_entry",
        "iom_0",
        "icm_grex1_entry",
        "icm_grex1_0",
    ]


def test_route_is_reversible(cluster):
    arm = cluster.arm("mhr_xz")
    there = highway_route(arm, "sim_exit", "gsm2_2")
    back = highway_route(arm, "gsm2_2", "sim_exit")
    assert back == list(reversed(there))


def test_route_uses_the_extra_nodes_that_break_up_long_x_moves(cluster):
    arm = cluster.arm("mhr_xz")
    # common_gsm_0/1 exist purely to keep single x moves under the wrap limit.
    route = highway_route(arm, "iom_entry", "gsm1_0")
    assert route == ["iom_entry", "gsm1_entry", "common_gsm_0", "common_gsm_1", "gsm1_0"]


def test_unknown_highway_node_raises(cluster):
    arm = cluster.arm("mhr_xz")
    with pytest.raises(PlanError, match="unknown highway node"):
        highway_route(arm, "iom_0", "nowhere")


# ---------------------------------------------------------------------------
# Arm tree walking
# ---------------------------------------------------------------------------

def test_arm_chain_returns_root_to_node_and_the_park_pose(cluster):
    arm = cluster.arm("mhr_xz")
    tree = arm.trees["rh_grex_1_icm_tree.drawer_0/1"]
    chain, park = arm_chain(tree, "drawer_opened")
    assert [n.name for n in chain] == ["enter_0", "enter_1", "drawer_opened"]
    assert park == "ORTHOGONAL_PARK"


def test_arm_chain_of_a_root_node_is_just_that_node(cluster):
    arm = cluster.arm("mhr_xz")
    tree = arm.trees["gsm_1_tree.eoat_holster"]
    chain, park = arm_chain(tree, "pick_place_node")
    assert [n.name for n in chain] == ["pick_place_node"]
    assert park == "ORTHOGONAL_PARK"


def test_arm_chain_rejects_an_unknown_node(cluster):
    arm = cluster.arm("mhr_xz")
    tree = arm.trees["gsm_1_tree.coupler"]
    with pytest.raises(PlanError, match="has no node"):
        arm_chain(tree, "syringe_z")


# ---------------------------------------------------------------------------
# Full plans
# ---------------------------------------------------------------------------

def test_full_plan_parks_first_then_rails_then_walks_the_arm(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "icm_grex1", "drawer_0", "drawer_opened", current_highway_node="sim_entry"
    )

    assert plan.steps[0].kind is StepKind.PARK
    assert plan.park_pose == "ORTHOGONAL_PARK"
    # Same J1 as Reset to PARK: the arm's configured park_base_angle (±90°).
    assert plan.steps[0].joints[0] == arm.park_base_angle_deg

    kinds = [s.kind for s in plan.steps]
    assert kinds.index(StepKind.PARK) < kinds.index(StepKind.HIGHWAY)
    assert kinds.index(StepKind.HIGHWAY) < kinds.index(StepKind.ARM)

    assert plan.highway_route == ["sim_entry", "iom_entry", "iom_0", "icm_grex1_entry"]
    assert [s.rail for s in rail_steps(plan)] == [
        RailPose(x=40.0, z=18.0),
        RailPose(x=300.0, z=18.0),
        RailPose(x=1005.0, z=18.0),
        RailPose(x=1010.0, z=18.0),
    ]
    assert [s.label for s in arm_steps(plan)] == [
        "node enter_0",
        "node enter_1",
        "node drawer_opened",
    ]
    assert plan.end_highway_node == "icm_grex1_entry"
    assert not plan.warnings


def test_arm_joints_come_straight_from_the_tree_yaml(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(arm, "gsm2", "bag_b_cartridge", "pick_place_node")
    tree = arm.trees["gsm_2_tree.bag_b_cartridge"]
    assert [s.joints for s in arm_steps(plan)] == [
        tree.nodes["enter_0"].joints,
        tree.nodes["enter_1"].joints,
        tree.nodes["pick_place_node"].joints,
    ]


def test_the_starting_node_is_not_re_commanded(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "iom1", "hotel_e_bag_cartridge", "enter_0", current_highway_node="iom_0"
    )
    # Starting on iom_0 whose rail pose is x=300 z=18; the first rail step must be the
    # target's own pose (x=310), not a repeat of where the arm already stands.
    assert rail_steps(plan)[0].rail == RailPose(x=310.0, z=18.0)


def test_a_duplicate_final_rail_move_is_dropped(cluster):
    arm = cluster.arm("mhr_xz")
    # gsm2_2 is at x=8240 z=818 and so is bag_b_cartridge, so no extra rail step.
    plan = plan_move(arm, "gsm2", "bag_b_cartridge", "pick_place_node")
    assert [s.kind for s in rail_steps(plan)][-1] is StepKind.HIGHWAY
    assert rail_steps(plan)[-1].rail == RailPose(x=8240.0, z=818.0)


def test_rail_targets_are_absolute_and_carry_every_axis(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(arm, "ncm1", "nc_cassette", "nc200")
    for step in rail_steps(plan):
        assert set(step.rail.axes()) == {"x", "z"}


def test_x_only_arm_never_produces_a_z_target(cluster):
    arm = cluster.arm("mhr_x")
    plan = plan_move(arm, "gsm1", "grex_5l_cartridge", "pick_place_node")
    assert [s.rail for s in rail_steps(plan)] == [RailPose(x=8130.0)]
    for step in rail_steps(plan):
        assert step.rail.z is None


def test_railless_arm_produces_no_rail_steps(cluster):
    arm = cluster.arm("mhr_u1")
    plan = plan_move(arm, "mhr1", "tuck_away", "tucked_away")
    assert rail_steps(plan) == []
    assert plan.highway_route == ["dummy"]
    assert [s.kind for s in plan.steps] == [StepKind.PARK, StepKind.ARM, StepKind.ARM]


def test_railless_arm_uses_the_upper_park_table(cluster):
    arm = cluster.arm("mhr_u1")
    plan = plan_move(arm, "mhr1", "tuck_away", "tucked_away")
    # UPPER ORTHOGONAL_PARK shoulder, versus -70.6 in the LOWER table.
    assert plan.steps[0].joints[1] == pytest.approx(-33.71)
    assert plan.steps[0].joints[0] == pytest.approx(90.0)


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------

def test_arm_only_mode_skips_the_highway(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm,
        "icm_grex1",
        "drawer_0",
        "drawer_opened",
        mode=Mode.ARM_ONLY,
        current_highway_node="sim_entry",
    )
    assert [s.kind for s in plan.steps if s.kind is StepKind.HIGHWAY] == []
    assert [s.rail for s in rail_steps(plan)] == [RailPose(x=1010.0, z=18.0)]
    assert len(arm_steps(plan)) == 3


def test_jump_mode_is_a_single_combined_step(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(arm, "icm_grex1", "drawer_0", "drawer_opened", mode=Mode.JUMP)
    assert len(plan.steps) == 1
    step = plan.steps[0]
    assert step.kind is StepKind.COMBINED
    assert step.rail == RailPose(x=1010.0, z=18.0)
    assert step.joints == arm.trees["rh_grex_1_icm_tree.drawer_0/1"].nodes[
        "drawer_opened"
    ].joints


@pytest.mark.parametrize("mode", ["full", Mode.FULL])
def test_mode_can_be_given_as_a_plain_string(cluster, mode):
    """Mode subclasses str, so a Qt combo box hands its value back as a bare str.

    Without coercion an identity comparison in the planner silently fell through to the
    arm-only branch, and Full navigation quietly skipped the highway.
    """
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "icm_grex1", "drawer_0", "drawer_opened", mode=mode, current_highway_node="sim_exit"
    )
    assert plan.mode is Mode.FULL
    assert len(plan.highway_route) == 5
    assert [s.kind for s in plan.steps].count(StepKind.HIGHWAY) == 4


def test_an_unknown_mode_is_rejected(cluster):
    with pytest.raises(ValueError):
        plan_move(cluster.arm("mhr_xz"), "icm_grex1", "drawer_0", "enter_0", mode="sideways")


def test_all_modes_end_at_the_same_pose_and_rail(cluster):
    arm = cluster.arm("mhr_xz")
    plans = [
        plan_move(arm, "gsm1", "coupler", "syringe_b", mode=mode, current_highway_node="iom_0")
        for mode in Mode
    ]
    ends = {(p.steps[-1].joints, last_rail(p)) for p in plans}
    assert len(ends) == 1, "modes disagree about where the move ends"


def test_start_rail_records_what_the_planner_assumed(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(arm, "gsm1", "coupler", "syringe_b", current_highway_node="iom_0")
    assert plan.start_rail == arm.highway["iom_0"].rail_pose


def test_the_rails_are_commanded_even_when_already_assumed_to_be_there(cluster):
    """mhr_x's only highway node sits at the same x as most of its targets.

    Trusting the assumption would emit no rail move and leave the rail wherever the
    station happened to have it, so the move has to be commanded regardless.
    """
    arm = cluster.arm("mhr_x")
    plan = plan_move(arm, "gsm1", "coupler", "syringe_a")
    assert plan.start_rail == RailPose(x=8500.0)
    assert [s.rail for s in rail_steps(plan)] == [RailPose(x=8500.0)]


def test_arm_only_and_jump_do_not_claim_a_start_rail(cluster):
    """Those modes ignore the highway, so where the rails were is genuinely unknown."""
    arm = cluster.arm("mhr_xz")
    for mode in (Mode.ARM_ONLY, Mode.JUMP):
        plan = plan_move(arm, "gsm1", "coupler", "syringe_b", mode=mode)
        assert plan.start_rail.is_empty()


def test_arm_only_always_commands_the_rails(cluster):
    """The rails' position is unknown when the highway is skipped, so never assume it."""
    arm = cluster.arm("mhr_xz")
    # gsm1.coupler sits exactly on its highway node gsm1_1, so a planner that assumed
    # the rails were already there would emit no rail move at all.
    plan = plan_move(arm, "gsm1", "coupler", "syringe_b", mode=Mode.ARM_ONLY)
    assert [s.rail for s in rail_steps(plan)] == [RailPose(x=8000.0, z=18.0)]


# ---------------------------------------------------------------------------
# Validation and warnings
# ---------------------------------------------------------------------------

def test_out_of_bounds_rail_target_warns(cluster, monkeypatch):
    arm = cluster.arm("mhr_xz")
    tight = replace(arm.rail_limits["x"], upper_mm=500.0)
    monkeypatch.setitem(arm.rail_limits, "x", tight)
    plan = plan_move(arm, "gsm1", "bag_b_cartridge", "pick_place_node")
    assert any("outside the x rail travel bounds" in w for w in plan.warnings)


def test_no_azula1_target_exceeds_the_rail_wrap_limit(cluster):
    """The highway is meant to keep every hop short; assert it actually does."""
    for arm_name in ("mhr_xz", "mhr_x"):
        arm = cluster.arm(arm_name)
        for module in arm.modules():
            for target in arm.targets(module):
                nav_target = arm.nav_target(module, target)
                tree = arm.tree(nav_target)
                node = next(iter(tree.nodes))
                plan = plan_move(arm, module, target, node)
                assert not [w for w in plan.warnings if "wrap limit" in w], (
                    f"{arm_name} {nav_target.label}: {plan.warnings}"
                )


def test_wrap_limit_warning_fires_when_the_highway_is_bypassed(cluster):
    arm = cluster.arm("mhr_xz")
    # ARM_ONLY starts from the target's own highway node, so nothing here should warn;
    # the guard is about hops, and this asserts the threshold is the documented one.
    assert MAX_RAIL_MOVE_MM == 8190.0
    plan = plan_move(arm, "gsm1", "bag_b_cartridge", "pick_place_node", mode=Mode.ARM_ONLY)
    assert not plan.warnings


def test_unknown_module_or_target_raises(cluster):
    arm = cluster.arm("mhr_xz")
    with pytest.raises(Exception, match="no nav target"):
        plan_move(arm, "nope", "nope", "nope")


def test_unknown_current_highway_node_raises(cluster):
    arm = cluster.arm("mhr_xz")
    with pytest.raises(PlanError, match="unknown current highway node"):
        plan_move(arm, "iom1", "coupler", "entry_0", current_highway_node="atlantis")


def test_a_bad_highway_node_is_reported_even_when_the_node_name_is_also_wrong(cluster):
    """The start node is validated first so the message names what the caller controls."""
    arm = cluster.arm("mhr_xz")
    with pytest.raises(PlanError, match="unknown current highway node"):
        plan_move(arm, "iom1", "coupler", "no_such_node", current_highway_node="atlantis")


# ---------------------------------------------------------------------------
# Pick / place (in and out)
# ---------------------------------------------------------------------------

def test_pick_place_walks_in_then_reverses_without_repeating_the_leaf(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "gsm2", "bag_b_cartridge", "pick_place_node", visit=Visit.PICK_PLACE
    )
    tree = arm.trees["gsm_2_tree.bag_b_cartridge"]
    inbound = [s for s in plan.steps if s.kind is StepKind.ARM]
    outbound = [s for s in plan.steps if s.kind is StepKind.EXIT]
    assert [s.joints for s in inbound] == [
        tree.nodes["enter_0"].joints,
        tree.nodes["enter_1"].joints,
        tree.nodes["pick_place_node"].joints,
    ]
    assert [s.label for s in outbound] == ["exit enter_1", "exit enter_0"]
    assert [s.joints for s in outbound] == [
        tree.nodes["enter_1"].joints,
        tree.nodes["enter_0"].joints,
    ]
    assert plan.steps[-1].kind is StepKind.PARK
    assert plan.steps[-1].joints[0] == arm.park_base_angle_deg


def test_pick_place_keeps_the_rails_at_the_target_while_retracting(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "icm_grex1", "drawer_0", "drawer_opened", visit="pick_place"
    )
    assert plan.visit is Visit.PICK_PLACE
    assert last_rail(plan) == RailPose(x=1010.0, z=18.0)
    for step in plan.steps:
        if step.kind in (StepKind.EXIT, StepKind.PARK) and step is not plan.steps[0]:
            assert step.rail.is_empty()


def test_exit_only_does_not_touch_the_rails_or_replay_the_inbound_walk(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm,
        "gsm2",
        "bag_b_cartridge",
        "pick_place_node",
        visit=Visit.EXIT,
        current_highway_node="iom_0",
    )
    assert [s.kind for s in plan.steps] == [StepKind.EXIT, StepKind.EXIT, StepKind.PARK]
    assert rail_steps(plan) == []
    assert plan.steps[-1].kind is StepKind.PARK


def test_jump_pick_place_hits_the_leaf_then_parks(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_move(
        arm, "icm_grex1", "drawer_0", "drawer_opened", mode=Mode.JUMP, visit=Visit.PICK_PLACE
    )
    assert [s.kind for s in plan.steps] == [StepKind.COMBINED, StepKind.PARK]
    assert plan.steps[0].joints == arm.trees["rh_grex_1_icm_tree.drawer_0/1"].nodes[
        "drawer_opened"
    ].joints


def test_every_pick_place_plan_visits_the_leaf_and_ends_at_park(cluster):
    planned = 0
    for arm in cluster.arms.values():
        for module in arm.modules():
            for target in arm.targets(module):
                nav_target = arm.nav_target(module, target)
                tree = arm.tree(nav_target)
                for node in tree.nodes:
                    plan = plan_move(arm, module, target, node, visit=Visit.PICK_PLACE)
                    assert any(s.joints == tree.nodes[node].joints for s in plan.steps)
                    if stays_at_node(node, target):
                        assert plan.steps[-1].joints == tree.nodes[node].joints
                        assert plan.steps[-1].kind is StepKind.ARM
                    else:
                        assert plan.steps[-1].kind is StepKind.PARK
                    planned += 1
    assert planned > 200


def test_tuck_away_ends_at_tucked_away_even_on_pick_place_visit(cluster):
    """tuck_away is a destination: reversing it would put the arm back at park."""
    for arm_name in ("mhr_xz", "mhr_x", "mhr_u1", "mhr_u2"):
        arm = cluster.arm(arm_name)
        module = next(
            m for m in arm.modules() if "tuck_away" in arm.targets(m)
        )
        plan = plan_move(arm, module, "tuck_away", "tucked_away", visit=Visit.PICK_PLACE)
        tree = arm.tree(arm.nav_target(module, "tuck_away"))
        assert [s.label for s in plan.steps if s.kind is StepKind.ARM][-1] == "node tucked_away"
        assert plan.steps[-1].joints == tree.nodes["tucked_away"].joints
        assert not [s for s in plan.steps if s.kind is StepKind.EXIT]


# ---------------------------------------------------------------------------
# Parking
# ---------------------------------------------------------------------------

def test_plan_park_is_one_arm_step_and_moves_no_rails(cluster):
    arm = cluster.arm("mhr_xz")
    plan = plan_park(arm, current_highway_node="iom_0")
    assert len(plan.steps) == 1
    assert plan.steps[0].rail.is_empty()
    # Uses the arm's own park_base_angle, not a target's base.
    assert plan.steps[0].joints[0] == arm.park_base_angle_deg
    assert plan.end_highway_node == "iom_0"


def test_navigation_park_matches_reset_to_park(cluster):
    arm = cluster.arm("mhr_xz")
    move = plan_move(
        arm, "icm_grex1", "drawer_0", "drawer_opened", current_highway_node="sim_entry"
    )
    reset = plan_park(arm)
    assert move.steps[0].joints == reset.steps[0].joints
    assert move.steps[0].joints[0] == pytest.approx(-90.0)


def test_plan_park_rejects_a_non_park_pose(cluster):
    with pytest.raises(PlanError, match="Not a park pose"):
        plan_park(cluster.arm("mhr_xz"), "enter_0")


# ---------------------------------------------------------------------------
# Whole-config sweep
# ---------------------------------------------------------------------------

def test_every_node_of_every_azula1_target_can_be_planned(cluster):
    planned = 0
    for arm in cluster.arms.values():
        for module in arm.modules():
            for target in arm.targets(module):
                nav_target = arm.nav_target(module, target)
                tree = arm.tree(nav_target)
                for node in tree.nodes:
                    plan = plan_move(arm, module, target, node)
                    assert plan.steps
                    assert plan.steps[-1].joints == tree.nodes[node].joints
                    planned += 1
    # Guards against the sweep silently passing because nothing was loaded.
    assert planned > 200
