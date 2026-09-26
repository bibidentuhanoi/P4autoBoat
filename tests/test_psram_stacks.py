"""Tasks whose stacks live in PSRAM never touch flash or NVS.

hw, 2026-09-26: the first WiFi-mode boot of the mission build could not fit
TrainingLog's 8 KB or the mission's 4 KB stack in the fragmented internal RAM
(ESP_ERR_NO_MEM, "no missions this boot"), so those two stacks moved to PSRAM
(runtime_schedule.c).  ESP-IDF's rule for a PSRAM stack (api-guides/
external-ram, "Task Stack Placement in External RAM"): any code path that
disables the cache -- flash erase/write, NVS, OTA -- must run on a task whose
stack is in internal RAM.  This walks every function a PSRAM-stacked task can
reach through the project's own sources and fails on any flash, partition,
NVS or OTA call, so a later change cannot quietly break that rule.
"""
import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAIN = ROOT / "main"

# Task entry -> the file that defines it.  Keep in step with the
# stack_in_psram entries of main/runtime_schedule.c (test_runtime_schedule.c
# pins that list).
PSRAM_TASK_ENTRIES = {
    "task_autonomy": MAIN / "autonomy.c",
    "training_log_task_fn": MAIN / "training_log_task.c",
}

FORBIDDEN = re.compile(r"^(nvs_|esp_partition_|esp_flash_|spi_flash_|esp_ota_|"
                       r"fs_save_|fs_load_|fs_init$)")
KEYWORDS = {"if", "for", "while", "switch", "return", "sizeof", "defined",
            "__attribute__", "_Static_assert", "static_assert"}


def _strip(src):
    """Comments and string/char literals out, so neither can fake a call."""
    src = re.sub(r"/\*.*?\*/", " ", src, flags=re.S)
    src = re.sub(r"//[^\n]*", " ", src)
    src = re.sub(r'"(\\.|[^"\\\n])*"', '""', src)
    return re.sub(r"'(\\.|[^'\\\n])*'", "''", src)


def _functions(path):
    """{name: body} for every function defined in a C file."""
    src = _strip(path.read_text(errors="replace"))
    out, depth, start, head_from = {}, 0, 0, 0
    for i, ch in enumerate(src):
        if ch == "{":
            if depth == 0:
                start = i
            depth += 1
        elif ch == "}":
            depth -= 1
            if depth == 0:
                head = src[head_from:start]
                m = re.search(r"\b([A-Za-z_]\w*)\s*\(([^()]|\([^()]*\))*\)\s*$", head)
                if m and not re.search(r"=\s*$", head) and m.group(1) not in KEYWORDS:
                    out[m.group(1)] = src[start:i + 1]
                head_from = i + 1
        elif ch == ";" and depth == 0:
            head_from = i + 1
    return out


def _project_functions():
    table = {}
    for path in sorted(MAIN.rglob("*.c")):
        for name, body in _functions(path).items():
            table.setdefault(name, []).append((path, body))
    return table


def _reachable(entry, entry_file, table):
    """Every project function reachable from entry; for a name defined in more
    than one file (static helpers), the entry's own file is preferred."""
    seen, calls, todo = set(), set(), [(entry, entry_file)]
    while todo:
        name, prefer = todo.pop()
        if name in seen:
            continue
        seen.add(name)
        defs = table.get(name, [])
        bodies = [b for p, b in defs if p == prefer] or [b for _, b in defs]
        for body in bodies:
            for callee in re.findall(r"\b([A-Za-z_]\w*)\s*\(", body):
                if callee in KEYWORDS:
                    continue
                calls.add(callee)
                if callee in table:
                    todo.append((callee, prefer))
    return seen, calls


def test_the_extractor_finds_the_nvs_path_it_must_reject():
    # The same walk from TrainingLog's INIT must see its NVS session counter --
    # proof the walker can see such a call at all.
    table = _project_functions()
    _, calls = _reachable("training_log_init", MAIN / "training_log_task.c", table)
    assert {"nvs_open", "nvs_set_u32", "nvs_commit"} <= calls


def test_psram_stacked_tasks_never_reach_flash_or_nvs():
    table = _project_functions()
    for entry, path in PSRAM_TASK_ENTRIES.items():
        assert entry in dict(_functions(path)), f"{entry} not found in {path.name}"
        reached, calls = _reachable(entry, path, table)
        bad = sorted(c for c in calls if FORBIDDEN.match(c))
        assert not bad, f"{entry} (PSRAM stack) reaches {bad}"
        assert len(reached) > 3, f"the walk from {entry} found almost nothing: {reached}"


def test_the_schedule_marks_exactly_these_tasks():
    src = (MAIN / "runtime_schedule.c").read_text()
    psram = set(re.findall(r"\[RUNTIME_TASK_(\w+)\]\s*=\s*\{[^}]*\btrue\s*,\s*true\s*\}", src)) | \
        set(re.findall(r"\[RUNTIME_TASK_(\w+)\]\s*=\s*\{[^}]*\bfalse\s*,\s*true\s*\}", src))
    assert psram == {"AUTONOMY", "TRAINING_LOG"}, psram
    for entry, path in PSRAM_TASK_ENTRIES.items():
        assert re.search(r"runtime_task_create\(RUNTIME_TASK_(AUTONOMY|TRAINING_LOG)\s*,\s*"
                         + entry + r"\b", path.read_text()), entry
