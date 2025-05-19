// [zigby -- an okay brainfuck interpreter in zig
// (c) 2025 mark joshwel <mark@joshwel.co>
// BSD Zero Clause License
//
// Permission to use, copy, modify, and/or distribute this software for any
// purpose with or without fee is hereby granted.
//
// THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES WITH
// REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
// AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT,
// INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM
// LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
// OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR
// PERFORMANCE OF THIS SOFTWARE.]

const std = @import("std");

const INSTRUCTION_CHARACTERS = "<>+-.,[]";

const Instruction = enum {
    OUTPUT,
    INPUT,
    POINTER_LEFT,
    POINTER_RIGHT,
    CELL_INCREMENT,
    CELL_DECREMENT,
    JUMP_FORWARD,
    JUMP_BACKWARD,
};

const InstructionPackage = packed struct(u32) {
    insts: u3 = 0,
    inst1: u3 = 0,
    inst2: u3 = 0,
    inst3: u3 = 0,
    inst4: u3 = 0,
    inst5: u3 = 0,
    inst6: u3 = 0,
    inst7: u3 = 0,
    count: u8 = 0, 
};

const InterpreterState = struct {
    instructions: std.ArrayList(InstructionPackage),
    instruction_pointer: u64 = 0,
    data_cells: std.ArrayList(u8),
    data_pointer: u64 = 0,
};

/// main function: process target file arg, read and pass to interpreter
pub fn main() void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const alloc = arena.allocator();

    // 1. get arguments
    var args_iter = std.process.argsWithAllocator(alloc) catch |err| {
        std.debug.print("zigby: internal error: {s}\n", .{@errorName(err)});
        return;
    };
    defer args_iter.deinit();

    var target: ?[]const u8 = null;
    var args_counter: u32 = 0;
    while (args_iter.next()) |arg| {
        if (args_counter == 1) {
            target = arg;
        }
        args_counter += 1;
    }

    if (args_counter < 1) {
        std.debug.print("zigby: error: no target file specified\n", .{});
        return;
    }

    if (target == null) {
        std.debug.print("zigby: internal error: target file string is null\n", .{});
        return;
    }

    // 2. open file
    const file = std.fs.cwd().openFile(target.?, std.fs.File.OpenFlags{}) catch |err| {
        std.debug.print("zigby: error: could not open file: {s}\n", .{@errorName(err)});
        return;
    };
    defer file.close();
    var target_buf_reader = std.io.bufferedReader(file.reader());
    var stream = target_buf_reader.reader();

    // 3. init program allocator and interpreter state
    const instructions_alloc = std.heap.page_allocator;
    var program_alloc_mgr = std.heap.DebugAllocator(.{}){};
    const program_alloc = program_alloc_mgr.allocator();
    defer _ = program_alloc_mgr.deinit();

    var state = InterpreterState{
        .instructions = std.ArrayList(InstructionPackage).init(program_alloc),
        .data_cells = std.ArrayList(u8).init(program_alloc),
    };
    defer state.instructions.deinit(instructions_alloc);
    defer state.data_cells.deinit();

    state.data_cells.resize(30000) catch |err| {
        std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {s}\n", .{@errorName(err)});
        return;
    };

    preprocess(&stream, &state.instructions, instructions_alloc) catch |err| {
        std.debug.print("zigby: internal error: could not preprocess instructions: {s}\n", .{@errorName(err)});
        return;
    };

    std.debug.print("preprocessed file to instruction length of {}\n", .{state.instructions.len});

    while (state.instruction_pointer <= state.instructions.len) {
        interpret(&state);
    }
}

fn preprocess(stream: anytype, instructions: *std.ArrayList(InstructionPackage), alloc: std.mem.Allocator) !void {
    var prog_line: u64 = 1;
    var prog_char: u64 = 0;

    while (true) {
        const current_char = stream.readByte() catch |err| {
            if (err == error.EndOfStream) {
                break;
            } else {
                return err;
            }
        };

        if (current_char == '\n') {
            prog_line += 1;
            prog_char = 0;
        }
        prog_char += 1;

        if (std.mem.indexOfScalar(u8, INSTRUCTION_CHARACTERS, current_char) == null) {
            continue;
        }

        if (running_character == 0) {
            running_character = current_char;
            running_character_count = 1;
        }

        if (running_character != current_char) {
            const running_instruction = instruction_map.get(&[_]u8{running_character});
            if (running_instruction == null) {
                @panic("running_character is not part of INSTRUCTION_CHARACTERS, even after a indexOfScalar check. this should not happen!");
            }
            
            while (running_character_count > 0) {
                // single
                if (running_character_count == 1) {
                    // std.debug.print("preprocess/rle: SINGLE '{c}'\n", .{running_character});
                    try instructions.append(alloc, Instruction{
                        .SINGLE = running_instruction.?,
                    });
                    running_character_count = 0;
                }
                // small (u8: 0-255)
                else if (running_character_count < 255) {
                    // std.debug.print("preprocess/rle: SMALL '{c}' ({}x)\n", .{running_character, running_character_count});
                    try instructions.append(alloc, Instruction{ .SMALL_REPEATED = SmallRepeatedInstruction{ .single = running_instruction.?, .count = @intCast(running_character_count) } });
                    running_character_count = 0;
                }
                // medium (u16: 0...65,535)
                else if (running_character_count < 65_535) {
                    // std.debug.print("preprocess/rle: MEDIUM '{c}' ({}x)\n", .{running_character, running_character_count});
                    try instructions.append(alloc, Instruction{ .MEDIUM_REPEATED = MediumRepeatedInstruction{ .single = running_instruction.?, .count = @intCast(running_character_count) } });
                    running_character_count = 0;
                }
                // large (u32: 0...4,294,967,295), or potentially larger
                else {
                    // std.debug.print("preprocess/rle: LARGE '{c}' ({}x)\n", .{running_character, running_character_count});
                    try instructions.append(alloc, Instruction{ .LARGE_REPEATED = LargeRepeatedInstruction{ .single = running_instruction.?, .count = running_character_count } });
                    running_character_count -= @min(running_character_count, 4_294_967_295);
                }
            }

            // update/reset the running character variables
            running_character = current_char;
            running_character_count = 1;
        } else {
            running_character_count += 1;
        }
    }
}

fn interpret(_: *InterpreterState) void {
    // https://esolangs.org/wiki/Brainfuck
    // >    Move the pointer to the right
    // <    Move the pointer to the left
    // +    Increment the memory cell at the pointer
    // -    Decrement the memory cell at the pointer
    // .    Output the character signified by the cell at the pointer
    // ,    Input a character and store it in the cell at the pointer
    // [    Jump past the matching ] if the cell at the pointer is 0
    // ]    Jump back to the matching [ if the cell at the pointer is nonzero
}
