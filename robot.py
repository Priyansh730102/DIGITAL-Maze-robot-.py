import sys
import traceback


def decide(sensors: dict, memory: dict) -> tuple[str, dict]:
    """Autonomous maze navigation algorithm using modified Depth-First Search.

    Guarantees full exploration and goal finding while strictly avoiding
    stdout output to comply with hackathon grading rules.
    """
    try:
        # ---------------------------------------------------------------------
        # 1. INITIALIZE MEMORY ON FIRST TICK
        # ---------------------------------------------------------------------
        if not memory:
            memory = {
                "x": 0,
                "y": 0,
                "dir": 0,  # 0: North (+Y), 1: East (+X), 2: South (-Y), 3: West (-X)
                "visited": {(0, 0): 1},  # Map (x, y) -> visit count
                "walls": {},  # Map (x, y) -> set of blocked absolute directions
                "stack": [],  # Stack of back-track absolute directions
                "last_action": None,
                "target_dir": None,  # Used during multi-tick turning
            }

        # Absolute direction unit vectors: N, E, S, W
        DIR_OFFSETS = [(0, 1), (1, 0), (0, -1), (-1, 0)]

        # ---------------------------------------------------------------------
        # 2. UPDATE STATE FROM PREVIOUS ACTION
        # ---------------------------------------------------------------------
        last_act = memory.get("last_action")
        curr_dir = memory["dir"]

        if last_act == "forward":
            # Check if we hit a wall instead of moving
            if sensors.get("accel_fwd", 0.0) == -2.0:
                # Mark absolute direction as blocked due to collision
                if (memory["x"], memory["y"]) not in memory["walls"]:
                    memory["walls"][(memory["x"], memory["y"])] = set()
                memory["walls"][(memory["x"], memory["y"])].add(curr_dir)
            else:
                # Movement succeeded: update local coordinates
                dx, dy = DIR_OFFSETS[curr_dir]
                memory["x"] += dx
                memory["y"] += dy

        elif last_act == "turn_right":
            memory["dir"] = (curr_dir + 1) % 4
        elif last_act == "turn_left":
            memory["dir"] = (curr_dir - 1) % 4

        curr_pos = (memory["x"], memory["y"])
        curr_dir = memory["dir"]

        # Increment visit count for current cell
        memory["visited"][curr_pos] = memory["visited"].get(curr_pos, 0) + 1

        # ---------------------------------------------------------------------
        # 3. RECORD WALL POSITIONS FROM DISTANCE SENSORS
        # ---------------------------------------------------------------------
        if curr_pos not in memory["walls"]:
            memory["walls"][curr_pos] = set()

        # Distance of 0 indicates an immediate wall blocking that relative side
        if sensors.get("dist_front", 1) == 0:
            memory["walls"][curr_pos].add(curr_dir)
        if sensors.get("dist_right", 1) == 0:
            memory["walls"][curr_pos].add((curr_dir + 1) % 4)
        if sensors.get("dist_left", 1) == 0:
            memory["walls"][curr_pos].add((curr_dir - 1) % 4)

        # Stop immediately if goal is flagged
        if sensors.get("at_goal", False):
            return "wait", memory

        # ---------------------------------------------------------------------
        # 4. DECISION ENGINE
        # ---------------------------------------------------------------------
        # If currently aligning heading towards a multi-step turn target
        if (
            memory.get("target_dir") is not None
            and memory["target_dir"] != curr_dir
        ):
            target = memory["target_dir"]
            # Decide shortest turn direction (right vs left)
            if (curr_dir + 1) % 4 == target:
                action = "turn_right"
            else:
                action = "turn_left"

            memory["last_action"] = action
            return action, memory

        memory["target_dir"] = None

        # Check all 4 absolute directions from current position
        candidates = []
        for abs_d in range(4):
            # Skip blocked directions
            if abs_d in memory["walls"][curr_pos]:
                continue

            dx, dy = DIR_OFFSETS[abs_d]
            nbr_pos = (curr_pos[0] + dx, curr_pos[1] + dy)
            visits = memory["visited"].get(nbr_pos, 0)
            candidates.append((visits, abs_d, nbr_pos))

        # Sort candidates by lowest visit count (prefer unvisited cells)
        candidates.sort(key=lambda item: item[0])

        chosen_dir = None
        if candidates and candidates[0][0] == 0:
            # Option A: Move to an unvisited cell
            chosen_dir = candidates[0][1]
            # Push return path to stack for backtracking
            opposite_dir = (chosen_dir + 2) % 4
            memory["stack"].append(opposite_dir)

        elif memory["stack"]:
            # Option B: Dead-end or loop detected — backtrack using stack
            chosen_dir = memory["stack"].pop()

        elif candidates:
            # Option C: Fallback to least-visited accessible neighbor
            chosen_dir = candidates[0][1]
        else:
            # Emergency wait step
            action = "wait"
            memory["last_action"] = action
            return action, memory

        # Execute action towards `chosen_dir`
        if chosen_dir == curr_dir:
            action = "forward"
        elif chosen_dir == (curr_dir + 1) % 4:
            action = "turn_right"
        elif chosen_dir == (curr_dir - 1) % 4:
            action = "turn_left"
        else:
            # 180 degree turn: start turning right, target set for next tick
            action = "turn_right"
            memory["target_dir"] = chosen_dir

        memory["last_action"] = action

        # Safe logging directly to stderr
        sys.stderr.write(
            f"[Tick] Pos={curr_pos} Dir={curr_dir} Choice={action}\n"
        )

        return action, memory

    except Exception as err:
        # Catch exceptions to prevent silent process crashes
        sys.stderr.write(f"Unhandled Exception: {err}\n")
        sys.stderr.write(traceback.format_exc())
        return "wait", memory


# =============================================================================
# DO NOT EDIT BLOCK BELOW THIS LINE
# =============================================================================
if __name__ == "__main__":
    import json

    # Signal readiness to the test harness
    print(json.dumps({"ready": True}))
    sys.stdout.flush()

    memory = {}
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            sensors = json.loads(line)
            action, memory = decide(sensors, memory)
            print(json.dumps({"action": action}))
            sys.stdout.flush()
        except Exception as e:
            sys.stderr.write(f"Runner error: {e}\n")
            break
