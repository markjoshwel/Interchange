# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "tqdm",
#     "ziglang",
# ]
# ///
from enum import Enum
from io import IOBase
from math import ceil
from pathlib import Path
from shutil import move, rmtree, which
from subprocess import run as invocate
from sys import executable as python
from sys import stderr
from tempfile import TemporaryDirectory
from time import time
from typing import Final
from argparse import ArgumentParser

from tqdm import tqdm

ZIG_SRC_COMMONS: Final[str] = """const std = @import("std");

pub fn State() type {    
    return struct {
        const Self = @This();
        
        outstream: std.fs.File.Writer,
        instream: std.fs.File.Reader,
        instream_buffer: std.ArrayList(u8),
        cells: std.ArrayList(u8),
        cc: usize,

        pub fn init(alloc: std.mem.Allocator) Self {
            var cells = std.ArrayList(u8).init(alloc);
            cells.resize(30000) catch |err| {
                std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {s}\\n", .{@errorName(err)});
                std.process.exit(255);
            };
            @memset(cells.items, 0);
            
            return .{
                .outstream = std.io.getStdOut().writer(),
                .instream = std.io.getStdIn().reader(),
                .instream_buffer = std.ArrayList(u8).init(alloc),
                .cells = cells,
                .cc = 0,
            };
        }
        
        pub fn deinit(self: *Self) void {
            self.instream_buffer.deinit();
            self.cells.deinit();
        }
    };
}

pub fn left(s: *State(), by: usize) void {
    if (s.cc == 0) {
        std.debug.print("error: pointer decrement out of bounds (@ cell {})\\n", .{s.cc});
        std.process.exit(1);
    }
    s.cc, _ = @subWithOverflow(s.cc, by);
}

pub fn right(s: *State(), by: usize) void {
    s.cc, _ = @addWithOverflow(s.cc, by);
    if (s.cc > s.cells.items.len) {
        s.cells.resize(s.cc + 1) catch |err| {
            std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {s}\\n", .{@errorName(err)});
            std.process.exit(255);
        };
    }
}

pub fn increment(s: *State(), by: usize) void {
    // s.cells.items[s.cc], _ = @addWithOverflow(s.cells.items[s.cc], by);
    var n = by;
    while (n > 0) {
        const d: u8 = @min(n, std.math.maxInt(u8));
        s.cells.items[s.cc] += d;
        n -= d;
    }
}

pub fn decrement(s: *State(), by: usize) void {
    // s.cells.items[s.cc], _ = @subWithOverflow(s.cells.items[s.cc], by);
    var n = by;
    while (n > 0) {
        const d = @min(n, std.math.maxInt(u8));
        s.cells.items[s.cc] -= d;
        n -= d;
    }
}

pub inline fn output(s: *State(), by: usize) void {
    @setEvalBranchQuota(4_294_967_295);
    for (0..by) |_| {
        s.outstream.writeByte(s.cells.items[s.cc]) catch |err| {
            std.debug.print("zigby: internal error: could not write byte out to stdout: {s}\\n", .{@errorName(err)});
            std.process.exit(255);
        };
    }
}

pub fn input(s: *State()) void {
    var inchar: u8 = 0;
    
    // 1. check if the instream buffer has length, if not read from stdin
    if (s.instream_buffer.items.len == 0) {
        // 04 is EOT=end of transmission,
        // the char is not consumed into the instream_buffer
        s.instream.streamUntilDelimiter(s.instream_buffer.writer(), 4, null) catch |report_err| {
            // EOT/EOF will not change the current cell
            if (report_err == error.EndOfStream) {
                return;
            }
            
            std.debug.print("zigby: internal error: could not read from stdin: {s}\\n", .{@errorName(report_err)});
            std.process.exit(255);
        };
        
        if (s.instream_buffer.items.len == 0) return;
    }
    // 2. if length is 1, get character then clear buffer
    if (s.instream_buffer.items.len == 1) {
        inchar = s.instream_buffer.items[0];
        defer s.instream_buffer.clearAndFree();
    // 3. if length > 1, get first character and remove it (fifo)
    } else if (s.instream_buffer.items.len > 1) {
        inchar = s.instream_buffer.orderedRemove(0);
    } else {
        unreachable;
    }
    
    s.cells.items[s.cc] = inchar;
}
"""

ZIG_SRC_MAIN_TEMPLATE: Final[str] = """const std = @import("std");
const c = @import("commons.zig");
{import_bodies}

pub fn main() !void {{
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    
    var state = c.State().init(arena.allocator());
    defer state.deinit();
    
    {call_bodies}
}}"""

ZIG_SRC_BODY_PREFIX: Final[str] = """const std = @import("std");
const c = @import("commons.zig");
pub fn body(s: *c.State()) void {
"""


class Operation(Enum):
    POINTER_LEFT = "<"
    POINTER_RIGHT = ">"
    CELL_INCREMENT = "+"
    CELL_DECREMENT = "-"
    OUTPUT = "."
    INPUT = ","
    JUMP_FORWARD = "["
    JUMP_BACKWARD = "]"
    OPT_SET_ZERO = "0"  # [+] / [-]


Instruction = tuple[Operation, int]


def bf_pass0_prepass(source: str) -> str:
    operations: list[str] = []

    in_a_scope: bool = False
    scope_depth_level: int = 0
    bf_source_index = -1

    filter_pbar = tqdm(
        total=len(source), desc="zigbpy: pass 0: scopeskip prepass", unit="chars"
    )

    while True:
        bf_source_index += 1
        filter_pbar.update(1)
        if (bf_source_index + 1) == len(source):
            print(
                "zigbpy: warning: scopeskip prepass exhausted the whole program, doing nothing",
                file=stderr,
            )
            exit(-1)
        char = source[bf_source_index]

        if in_a_scope:
            if char not in "[]":
                continue
            if char == "[":
                scope_depth_level += 1
            if char == "]":
                scope_depth_level -= 1
                if scope_depth_level == 0:
                    in_a_scope = False
        else:
            if char in "<>+-.,":
                # operations.append(char)
                break
            if char not in "[]":
                continue
            if char == "[":
                in_a_scope = True
                scope_depth_level = 1
            if char == "]":
                print(
                    "\nzigbpy: error: unmatched closing bracket during scopeskip prepass",
                    file=stderr,
                )
                exit(-1)

    depths: dict[int, tuple[int, int]] = {
        # depth_number: [entrance index, exit index],
    }
    entered_at: int = -1
    filter_pbar.set_description(f"zigbpy: pass 0: filtering from [{bf_source_index}:]")

    for idx, char in enumerate(
        source[bf_source_index:],
        start=bf_source_index,
    ):
        filter_pbar.update(1)

        if char not in "[]<>+-.,":
            continue

        if char == "[":
            scope_depth_level += 1
            entered_at = idx
            depths[scope_depth_level] = (idx, -1)

        if char == "]":
            depths[scope_depth_level] = (
                entered_at,
                idx,
            )
            scope_depth_level -= 1

        operations.append(char)

    if scope_depth_level != 0:
        print(
            f"\nzigbpy: error: there are unmatched brackets (ended at a scope depth level of {scope_depth_level})",
            file=stderr,
        )
        found = False
        for depth, (start, end) in depths.items():
            if (start == -1) or (end == -1):
                print(
                    f"... note: depth {depth} - program index {start} -> program index {end} [unmatched]",
                    file=stderr,
                )
                found = True

        if not found:
            for depth, (start, end) in depths.items():
                print(
                    f"... note: depth {depth} - program index {start} -> program index {end}",
                    file=stderr,
                )

        exit(-1)

    filter_pbar.close()
    source = "".join(operations)

    # deadcode elim
    for template, response in (
        deadcode_pbar := tqdm(
            (
                ("<>", ""),
                ("+-", ""),
                ("-+", ""),
                ("[+]", "0"),
                ("[-]", "0"),
            ),
            desc="zigbpy: pass 0: deadcode elim + lhf optim",
            unit="opts",
        )
    ):
        source = source.replace(template, response)
    else:
        deadcode_pbar.close()

    return source


def bf_pass1_pack(bf_source: str) -> list[Instruction]:
    packed_insts: list[Instruction] = []

    last_char: str = bf_source[0]
    count: int = 1

    for index, char in (
        pbar := tqdm(
            enumerate(bf_source[1:], start=1),
            desc="zigbpy: pass 1: naive rle packing",
            total=len(bf_source),
            unit="chars",
        )
    ):
        if char == last_char:
            count += 1
        else:
            packed_insts.append((Operation(last_char), count))
            last_char = char
            count = 1
    else:
        packed_insts.append((Operation(last_char), count))
        pbar.close()

    return packed_insts


def zig_emit_instruction(instruction: Instruction) -> str:
    match instruction:
        # pub fn left(s: *State(), by: usize) void {
        case (Operation.POINTER_LEFT, count):
            return f"    c.left(s, {count});\n"

        # pub fn right(s: *State(), by: usize) void {
        case (Operation.POINTER_RIGHT, count):
            return f"    c.right(s, {count});\n"

        # pub fn increment(s: *State(), by: usize) void {
        case (Operation.CELL_INCREMENT, count):
            return f"    c.increment(s, {count});\n"

        # pub fn decrement(s: *State(), by: usize) void {
        case (Operation.CELL_DECREMENT, count):
            return f"    c.decrement(s, {count});\n"

        # pub fn input(s: *State()) void {
        case (Operation.INPUT, count):
            return "    c.input(s);\n" * count

        # pub inline fn output(s: *State(), by: usize) void {
        case (Operation.OUTPUT, count):
            return f"    c.output(s, {count});\n"

        case (Operation.JUMP_FORWARD, count):
            return (
                "    if (s.cells.items[s.cc] != 0) {\n        while (true) {\n"
            ) * count

        case (Operation.JUMP_BACKWARD, count):
            return (
                "            if (s.cells.items[s.cc] == 0) break;\n        }\n    }\n"
            ) * count

        case (Operation.OPT_SET_ZERO, _):
            return "    s.cells.items[s.cc] = 0;\n"

        case _:
            raise NotImplementedError(instruction)


def zig_build_source(
    name: str, construction_dir: Path, insts: list[Instruction]
) -> bool:
    """
    chunk up insts into 100k LOC each, and then send each off into a process
    to be built into its own file
    """

    split_every = 150_000

    # build modules

    modules: list[str] = []
    pbar = tqdm(
        desc=f"zigbpy: zig_build_source: emitting module body 1/{ceil(len(insts) / split_every)}...",
        total=len(insts),
    )

    module: IOBase = construction_dir.joinpath(f"{name}-body1.zig").open(
        "w", encoding="utf-8"
    )
    module.write(ZIG_SRC_BODY_PREFIX)
    modules.append(f"{name}-body1.zig")

    running_inst_count = 0
    running_scope_depth = 0

    for op, count in insts:
        # trackers
        pbar.update(1)
        running_inst_count += 1
        if op == Operation.JUMP_FORWARD:
            running_scope_depth += 1
        elif op == Operation.JUMP_BACKWARD:
            running_scope_depth -= 1
        else:
            # its been a bit, is it safe to break into another module
            if (running_inst_count >= 150_000) and (running_scope_depth == 0):
                module.write("}\n")
                module.flush()
                module.close()
                running_inst_count = 0
                running_scope_depth = 0

                new_module_name = f"{name}-body{len(modules) + 1}.zig"
                module = construction_dir.joinpath(new_module_name).open(
                    "w", encoding="utf-8"
                )
                modules.append(new_module_name)
                module.write(ZIG_SRC_BODY_PREFIX)
                pbar.set_description(
                    f"zigbpy: zig_build_source: emitting module body {len(modules)}/{ceil(len(insts) / split_every)}...",
                )

        # write to current module
        module.write(zig_emit_instruction((op, count)))

    else:
        module.write("}\n")
        module.flush()
        module.close()

    pbar.close()

    # build commons and main file

    construction_dir.joinpath("commons.zig").write_text(ZIG_SRC_COMMONS)

    import_bodies: list[str] = []
    call_bodies: list[str] = []

    for mod_idx, mod_file in enumerate(modules, start=1):
        import_bodies.append(f'const b{mod_idx} = @import("{mod_file}");')
        call_bodies.append(f"    b{mod_idx}.body(&state);")

    construction_dir.joinpath(f"{name}.zig").write_text(
        ZIG_SRC_MAIN_TEMPLATE.format(
            import_bodies="\n".join(import_bodies), call_bodies="\n".join(call_bodies)
        )
    )

    return True


def zig_compile_dir(
    name: str,
    construction_dir: Path,
    zig_inovcation: str | None = None,
    platform_target: str | None = None,
):
    # check if zig is present on system
    zig: list[str] = ["zig"]
    if zig_inovcation is not None:
        zig_inovcation.strip().split()
    elif (which_zig := which("zig")) is not None:
        zig = [which_zig]
    else:
        zig = [python, "-m", "ziglang"]

    target: list[str] = []
    if platform_target is not None:
        target.extend(["-target", platform_target])

    build_cp = invocate(
        invocation := [
            *zig,
            "build-exe",
            f"{name}.zig",
            "-O",
            "ReleaseSmall",
            "-fstrip",
            "-fsingle-threaded",
            *target,
        ],
        cwd=construction_dir,
    )
    if build_cp.returncode != 0:
        # we're not hiding zig compiler output, so just use whatever has
        # already been printed to stderr)
        print(
            f"zigbpy: failed to compile {name}, see zig compiler output above\n",
            f"... note: invocation={invocation}",
            file=stderr,
        )
        return False

    program = construction_dir.joinpath(f"{name}.zig")

    if program.with_suffix(".exe").exists():
        move(program.with_suffix(".exe"), Path.cwd().joinpath(f"{program.stem}.exe"))
        return True

    elif program.with_suffix("").exists():
        move(program.with_suffix(""), Path.cwd().joinpath(f"{program.stem}"))
        return True

    else:
        print(
            "zigbpy: could not move out executable, was not found in temp build directory",
            file=stderr,
        )
        return False

    return False


def generate_time_elapsed_string(time_taken: float) -> str:
    """generates a human-readable time-elapsed string from a time-taken float"""
    hours = int(time_taken // 3600)
    minutes = int(time_taken % 3600 // 60)
    seconds = time_taken % 60
    if time_taken > 3600:
        return f"{hours}h {minutes}′ {seconds:.1f}″"
    elif time_taken > 60:
        return f"{minutes}′ {seconds:.3f}″"
    else:
        return f"{time_taken:.3f}″"


def handle_args() -> tuple[str, str | None, str | None, bool, bool, bool]:
    parser = ArgumentParser(description="a brainfuck to zig compiler")
    parser.add_argument("target", type=Path, help="brainfuck source file")
    parser.add_argument(
        "--platform",
        type=str,
        help="specify a target zig-supported platform tuple for compilation",
        default=None,
    )
    parser.add_argument(
        "--zig",
        type=str,
        help="specify a custom zig invocation command",
        default=None,
    )
    parser.add_argument(
        "--keep",
        help="keep the temporary zig construction directory",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--dont-compile",
        help="builds the zig code but does not compile it",
        action="store_true",
        default=False,
    )
    parser.add_argument(
        "--run-after",
        help="runs the executable after compiling it",
        action="store_true",
        default=False,
    )

    args = parser.parse_args()

    target: str
    platform_target: str | None
    zig_invocation: str | None
    keep: bool
    dont_compile: bool
    run_after: bool

    target = args.target
    platform_target = args.platform
    zig_invocation = args.zig
    keep = args.keep
    dont_compile = args.dont_compile
    run_after = args.run_after

    return target, platform_target, zig_invocation, keep, dont_compile, run_after


def main() -> None:
    (_target, platform_target, zig_invocation, keep, dont_compile, run_after) = (
        handle_args()
    )
    target = Path(_target)

    if not (target.exists() and target.is_file()):
        print(f"zigbpy: error: {target} does not exist or is not a file", file=stderr)
        exit(1)

    global_time_start = time()
    bf_source_raw = target.read_text(encoding="utf-8")

    bf_source_filtered = bf_pass0_prepass(bf_source_raw)
    del bf_source_raw

    bf_insts = bf_pass1_pack(bf_source_filtered)
    del bf_source_filtered

    ok: bool = True
    with TemporaryDirectory(delete=False) as temp_dir:
        try:
            construction_dir = Path(temp_dir)
            ok = zig_build_source(target.stem, construction_dir, bf_insts)
            if not dont_compile:
                ok = zig_compile_dir(
                    target.stem,
                    construction_dir,
                    zig_inovcation=zig_invocation,
                    platform_target=platform_target,
                )
        except KeyboardInterrupt:
            print("zigbpy: interrupted", file=stderr)
            exit(2)

    global_time_end = time()
    print(
        f"zigbpy: time elapsed: {generate_time_elapsed_string(global_time_end - global_time_start)}",
        file=stderr,
    )

    if ok and (not keep):
        rmtree(temp_dir)
    else:
        print(
            f"... note: temporary construction dir {construction_dir} will not be deleted",
            file=stderr,
        )

    del bf_insts

    if run_after:
        output = Path.cwd().joinpath(target.stem)
        output = output if output.exists() else output.with_suffix(".exe")
        if not output.exists():
            print(
                "zigbpy: internal error: could not run output executable (could not find it)",
                file=stderr,
            )
            exit(1)
        cp = invocate([output])
        exit(cp.returncode)
    else:
        exit(0 if ok else 5)


if __name__ == "__main__":
    main()
