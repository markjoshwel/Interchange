# zigbpy: a brainfuck to zig compiler
# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "ziglang",
# ]
# ///


from functools import partial
from enum import Enum
from pathlib import Path
from shutil import move, which
from subprocess import run as invocate
from sys import argv
from sys import executable as python
from sys import stderr
from tempfile import TemporaryDirectory
from time import time
from typing import Callable, NamedTuple, TypeVar


class Operation(Enum):
    """
    const LOOKUP_INSTRUCTION_CHAR_TO_ENUM = blk: {
        var lookup = [_]?Instruction{null} ** 256;
        lookup['<'] = Instruction.POINTER_LEFT;
        lookup['>'] = Instruction.POINTER_RIGHT;
        lookup['+'] = Instruction.CELL_INCREMENT;
        lookup['-'] = Instruction.CELL_DECREMENT;
        lookup['.'] = Instruction.OUTPUT;
        lookup[','] = Instruction.INPUT;
        lookup['['] = Instruction.JUMP_FORWARD;
        lookup[']'] = Instruction.JUMP_BACKWARD;
        break :blk lookup;
    };
    """

    POINTER_LEFT = "<"
    POINTER_RIGHT = ">"
    CELL_INCREMENT = "+"
    CELL_DECREMENT = "-"
    OUTPUT = "."
    INPUT = ","
    JUMP_FORWARD = "["
    JUMP_BACKWARD = "]"
    OPT_SET_ZERO = "0"  # [+] / [-]


class Instruction(NamedTuple):
    inst: Operation
    a: int
    b: int


def bf_pass0_prepass(source: str) -> str:
    operations: list[str] = []

    in_a_scope: bool = False
    scope_depth_level: int = 0
    bf_source_index = -1

    while True:
        bf_source_index += 1
        if (bf_source_index + 1) == len(source):
            print(
                "zigbpy: warning: scopeskip prepass exhausted the whole program, doing nothing"
            )
            exit(-1)
        char = source[bf_source_index]

        if in_a_scope:
            if char in "<>+-.,":
                break
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

    # {depth_number: [entrance index, exit index]}
    # scope_depth_level = 0
    depths: dict[int, tuple[int, int]] = {}
    entered_at: int = -1
    for idx, char in enumerate(
        source[bf_source_index:],
        start=bf_source_index,
    ):
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
                    f"... note: depth {depth} - program index {start} -> program index {end} [unmatched]"
                )
                found = True

        if not found:
            for depth, (start, end) in depths.items():
                print(
                    f"... note: depth {depth} - program index {start} -> program index {end}"
                )

        exit(-1)

    source = "".join(operations)

    # deadcode elim
    source = source.replace("<>", "")
    source = source.replace("+-", "")
    source = source.replace("-+", "")
    source = source.replace("[+]", "0")
    source = source.replace("[-]", "0")

    return source


def bf_pass1_pack(bf_source: str) -> list[Instruction]:
    packed_insts: list[Instruction] = []

    last_char: str = bf_source[0]
    count: int = 1

    for index, char in enumerate(bf_source[1:], start=1):
        if char == last_char:
            count += 1
        else:
            packed_insts.append(Instruction(Operation(last_char), count, 0))
            last_char = char
            count = 1
    else:
        packed_insts.append(Instruction(Operation(last_char), count, 0))

    return packed_insts


def _zig_emit_unit(unit: Instruction) -> list[str]:
    emitted_source: list[str] = []
    emitted_source.append(f"// {unit.inst.value * unit.a}")

    match unit:
        case Instruction(Operation.POINTER_LEFT, by, _):
            emitted_source.extend(
                f"""
                if (cc == 0) {{
                    std.debug.print("error: pointer decrement out of bounds (@ cell {{}})\\n", .{{cc}});
                    std.process.exit(1);
                }}
                cc -= {by};
                """.strip().splitlines(),
            )

        case Instruction(Operation.POINTER_RIGHT, by, _):
            emitted_source.extend(
                f"""
                cc += {by};
                if (cc > cells.items.len) {{
                    cells.resize(cc + 1) catch |err| {{
                        std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {{s}}\\n", .{{@errorName(err)}});
                        std.process.exit(255);
                    }};
                }}
                """.strip().splitlines(),
            )

        case Instruction(Operation.CELL_INCREMENT, by, _):
            emitted_source.extend(
                f"""
                cells.items[cc], _ = @addWithOverflow(cells.items[cc], {by});
                """.strip().splitlines(),
            )

        case Instruction(Operation.CELL_DECREMENT, by, _):
            emitted_source.extend(
                f"""
                cells.items[cc], _ = @subWithOverflow(cells.items[cc], {by});
                """.strip().splitlines(),
            )

        case Instruction(Operation.OUTPUT, by, _):
            holders = "{c}" * by
            params = ", ".join("cells.items[cc]" for _ in range(by))
            emitted_source.extend(
                f"""
                outstream.print("{holders}", .{{{params}}}) catch |err| {{
                    std.debug.print("zigby: internal error: could not print out: {{s}}\\n", .{{@errorName(err)}});
                    std.process.exit(255);
                }};
                """.strip().splitlines(),
            )

        case Instruction(Operation.INPUT, by, _):
            emitted_source.extend(
                """
                cells.items[cc] = input: {
                    var inchar: u8 = 0;
                    // 1. check if the instream buffer has length, if not read from stdin
                    if (instream_buffer.items.len == 0) {
                        instream.streamUntilDelimiter(instream_buffer.writer(), 10, null) catch {
                            break :input 0;
                        };
                        if (instream_buffer.items.len == 0) break :input 0;
                    }
                    // 2. if length is 1, get character then clear buffer
                    if (instream_buffer.items.len == 1) {
                        inchar = instream_buffer.items[0];
                        defer instream_buffer.clearAndFree();
                    // 3. if length > 1, get first character and remove it (fifo)
                    } else if (instream_buffer.items.len > 1) {
                        inchar = instream_buffer.orderedRemove(0);
                    } else {
                        unreachable;
                    }
                    break :input inchar;
                };
                """.strip().splitlines()
            )

        case Instruction(Operation.JUMP_FORWARD, by, _):
            for _ in range(by):
                emitted_source.extend(
                    """
                    if (cells.items[cc] != 0) {
                        while (true) {
                    """.strip().splitlines()
                )

        case Instruction(Operation.JUMP_BACKWARD, by, _):
            for _ in range(by):
                emitted_source.extend(
                    """
                            if (cells.items[cc] == 0) break;
                        }
                    }
                    """.strip().splitlines()
                )

        case Instruction(Operation.OPT_SET_ZERO, _, _):
            emitted_source.extend(
                """
                        cells.items[cc] = 0;
                """.strip().splitlines()
            )

        case _:
            raise NotImplementedError(unit.inst)

    emitted_source.append("")
    return emitted_source


def zig_emit(instructions: list[Instruction]) -> str:
    zig_source_header: list[str] = [
        """const std = @import("std");""",
        """pub fn main() !void {""",
        """    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);""",
        """    const allocator = arena.allocator();""",
        """    var cells = std.ArrayList(u8).init(allocator);""",
        """    defer cells.deinit();""",
        """    cells.resize(30000) catch |err| {""",
        """        std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {s}\\n", .{@errorName(err)});""",
        """        std.process.exit(255);""",
        """    };""",
        """    @memset(cells.items, 0);""",
    ]
    zig_source_body: list[str] = []

    for i in instructions:
        zig_source_body.extend(_zig_emit_unit(i))
    else:
        zig_source_body.append("}")

    cc_indexed: bool = False
    cc_mutated: bool = False
    outstream_accessed: bool = False
    instream_accessed: bool = False

    for line in zig_source_body:
        if "[cc]" in line:
            cc_indexed = True

        if ("cc +=" in line) or ("cc -=" in line):
            cc_mutated = True

        if ("outstream" in line) and (not outstream_accessed):
            zig_source_header.append(
                """    var outstream = std.io.getStdOut().writer();"""
            )
            outstream_accessed = True

        if (("instream" in line) or ("instream_buffer" in line)) and (
            not instream_accessed
        ):
            zig_source_header.extend(
                (
                    """    var instream = std.io.getStdIn().reader();\n"""
                    """    var instream_buffer = std.ArrayList(u8).init(allocator);\n"""
                    """    defer instream_buffer.deinit();"""
                ).splitlines()
            )
            instream_accessed = True

    else:
        if cc_mutated:
            zig_source_header.append("var cc: usize = 0;")
        elif cc_indexed:
            zig_source_header.append("const cc: usize = 0;")

        zig_source_header.append("\n")

    return "\n".join(zig_source_header + zig_source_body)


def zig_compile(source: str, name: str) -> bool:
    """
    returns true if sucessful, else compiler output has already been printed
    out to stderr
    """
    with TemporaryDirectory() as temp_dir:
        temp = Path(temp_dir)
        program = temp.joinpath(f"{name}.zig")
        program.write_text(source)

        # check if zig is present on system
        zig_which = which("zig")
        zig = [zig_which] if zig_which is not None else [python, "-m", "ziglang"]

        fmt_cp = invocate(
            [
                *zig,
                "fmt",
                str(program.absolute()),
            ],
            cwd=temp,
        )
        if fmt_cp.returncode != 0:
            print(
                f"zigbpy: failed to format {program.name}, meaning the emitted code is not grammatically correct, see zig compiler output above",
                file=stderr,
            )
            return False

        build_cp = invocate(
            [
                *zig,
                "build-exe",
                str(program),
                "-O",
                "ReleaseSmall",
            ],
            cwd=temp,
        )
        if build_cp.returncode != 0:
            # we're not hiding zig compiler output, so just use whatever has
            # already been printed to stderr)
            print(
                f"zigbpy: failed to compile {program.name}, see zig compiler output above",
                file=stderr,
            )
            return False

        if program.with_suffix(".exe").exists():
            move(
                program.with_suffix(".exe"), Path.cwd().joinpath(f"{program.stem}.exe")
            )
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


def main() -> None:
    target: Path

    match argv:
        case [_, target_str]:
            target = Path(target_str)

        case _:
            print("usage: zigby [file]", file=stderr)
            exit(1)

    if not target.exists():
        print(f"zigby: error: file '{target}' does not exist", file=stderr)
        exit(2)

    T = TypeVar("T")

    def _run(func: Callable[..., T], what: str, show_done: bool = True) -> T:
        if show_done:
            stderr.write(f"zigbpy: {what}...")
        else:
            stderr.write(f"zigbpy: {what}\n")
        stderr.flush()

        start_time = time()
        result = func()
        end_time = time()

        if show_done:
            stderr.write(
                f" done in {generate_time_elapsed_string(end_time - start_time)}\n"
            )
        else:
            stderr.write(
                f"...done in {generate_time_elapsed_string(end_time - start_time)}\n"
            )
        stderr.flush()

        return result

    bf_source: str = _run(partial(target.read_text), what="reading text")
    bf_source = _run(partial(bf_pass0_prepass, bf_source), what="preprocessing text")

    bf_instructions = _run(
        partial(bf_pass1_pack, bf_source), what="packing instructions"
    )
    del bf_source

    zig_source = _run(partial(zig_emit, bf_instructions), what="emitting zig code")
    del bf_instructions
    if (not zig_compile(zig_source, target.stem)) or True:
        emergency_output = Path.cwd().joinpath(f"{target.stem}.zig")
        print(f"... note: transpiled zig code has been outputted to {emergency_output}")
        emergency_output.write_text(zig_source)
        invocate(
            [
                *(
                    [zig_which]
                    if (zig_which := which("zig")) is not None
                    else [python, "-m", "ziglang"]
                ),
                "fmt",
                emergency_output,
            ],
        )
        exit(-1)


if __name__ == "__main__":
    main()
