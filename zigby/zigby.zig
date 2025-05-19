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

const SingleInstruction = enum {
    POINTER_RIGHT,
    POINTER_LEFT,
    CELL_INCREMENT,
    CELL_DECREMENT,
    OUTPUT,
    INPUT,
    JUMP_FORWARD,
    JUMP_BACKWARD,
};

const SmallRepeatedInstruction = struct {
    single: SingleInstruction,
    count: u8,
};

const MediumRepeatedInstruction = struct {
    single: SingleInstruction,
    count: u16,
};

// no way we need more than 4,294,967,295 repeated bf instructions
const LargeRepeatedInstruction = struct {
    single: SingleInstruction,
    count: u32,
};

const Instruction = union(enum) {
    SINGLE: SingleInstruction,
    SMALL_REPEATED: SmallRepeatedInstruction,
    MEDIUM_REPEATED: MediumRepeatedInstruction,
    LARGE_REPEATED: LargeRepeatedInstruction,
};

const InterpreterState = struct {
    instructions: std.MultiArrayList(Instruction),
    instruction_pointer: u64 = 0,
    data_cells: std.ArrayList(u8),
    data_pointer: u64 = 0,
};

// pub fn interpret(_inst: u8, state: *ProgramState) bool {
//     var inst = _inst;
//     var inst_slice: []const u8 = &[_]u8{_inst};
//     // are we jumping past a scope? if so, keep track of nestled scopes so we
//     // can resume where we want to
//     if (state.scope_jump_past) {
//         if (std.mem.eql(u8, inst_slice, "[")) {
//             state.jump_level += 1;
//         } else if (std.mem.eql(u8, inst_slice, "]")) {
//             state.jump_level -= 1;
//             if (state.jump_level == 0) {
//                 state.scope_jump_past = false;
//             }
//         }
//         return true;
//     }
//     // are we currently reinterpreting a scope? (is rewind_counter > 0?)
//     // if so, override inst and inst_slice to the
//     // (scope_instructions.len - rewind_counter)th index of scope_instructions
//     // and use that as the current inst
//     // (interpret the buffered scope until we exit it)
//     if (state.rewind_counter > 0) {
//         if ((state.rewind_counter > state.scope_instructions.items.len) and (state.scope_instructions.items.len >= (state.scope_instructions.items.len - state.rewind_counter))) {
//             std.debug.print("zigby: internal error: attempting to rewind further ({}) than the buffered scope instructions ({})\n", .{ state.rewind_counter, state.scope_instructions.items.len });
//             return false;
//         }
//         inst = state.scope_instructions.items[state.scope_instructions.items.len - state.rewind_counter];
//         inst_slice = &[_]u8{inst};
//         // std.debug.print("debug: rewinding to instruction: {c}\n", .{inst});
//         state.rewind_counter -= 1;
//     }
//     // if we're not reinterpreting a scope, and the scope level is not 0, we
//     // should be keeping track of the scope instructions in case we need to
//     // rewind to them later
//     else if (state.scope_depth != 0) {
//         state.scope_instructions.append(inst) catch |err| {
//             std.debug.print("zigby: internal error: failed to append instruction to scope instructions: {}\n", .{err});
//             return false;
//         };
//     }
//     // standard program execution
//     // https://esolangs.org/wiki/Brainfuck
//     // >   Move the pointer to the right
//     if (std.mem.eql(u8, inst_slice, ">")) {
//         if (state.cell_counter >= state.data_cells.len) {
//             std.debug.print("error: pointer increment out of bounds (@ cell {})\n", .{state.cell_counter});
//             return false;
//         }
//         state.cell_counter += 1;
//     }
//     // <   Move the pointer to the left
//     if (std.mem.eql(u8, inst_slice, "<")) {
//         if (state.cell_counter == 0) {
//             std.debug.print("error: pointer decrement out of bounds (@ cell {})\n", .{state.cell_counter});
//             return false;
//         }
//         state.cell_counter -= 1;
//     }
//     // +   Increment the memory cell at the pointer
//     if (std.mem.eql(u8, inst_slice, "+")) {
//         const result, _ = @addWithOverflow(state.data_cells[state.cell_counter], 1);
//         state.data_cells[state.cell_counter] = result;
//     }
//     // -   Decrement the memory cell at the pointer
//     if (std.mem.eql(u8, inst_slice, "-")) {
//         const result, _ = @subWithOverflow(state.data_cells[state.cell_counter], 1);
//         state.data_cells[state.cell_counter] = result;
//     }
//     // .   Output the character signified by the cell at the pointer
//     if (std.mem.eql(u8, inst_slice, ".")) {
//         std.io.getStdOut().writer().print("{c}", .{state.data_cells[state.cell_counter]}) catch |err| {
//             std.debug.print("zigby: internal error: could not output cell {}: {s}\n", .{ state.cell_counter, @errorName(err) });
//             return false;
//         };
//     }
//     // ,   Input a character and store it in the cell at the pointer
//     if (std.mem.eql(u8, inst_slice, ",")) {
//         state.data_cells[state.cell_counter] = std.io.getStdIn().reader().readByte() catch |err| {
//             std.debug.print("zigby: internal error: could not input cell {}: {s}\n", .{ state.cell_counter, @errorName(err) });
//             return false;
//         };
//     }
//     // [   Jump past the matching ']' if the cell at the pointer is 0
//     if (std.mem.eql(u8, inst_slice, "[")) {
//         if (state.data_cells[state.cell_counter] == 0) {
//             // we're zero, we'll skip this current scope
//             state.jump_level = 1;
//             state.scope_jump_past = true;
//         } else {
//             // we're nonzero, we'll keep executing
//             state.scope_depth += 1;
//             state.scope_instructions.append(inst) catch |err| {
//                 std.debug.print("zigby: internal error: failed to append instruction to scope instructions: {}\n", .{err});
//                 return false;
//             };
//         }
//     }
//     // ]   Jump back to the matching '[' if the cell at the pointer is nonzero
//     if (std.mem.eql(u8, inst_slice, "]")) {
//         if (state.data_cells[state.cell_counter] == 0) {
//             // we're zero, we're getting out
//             state.scope_depth -= 1;
//             // are we back at the top? if so, it's now safe to clear the
//             // instruction basket
//             if (state.scope_depth == 0) {
//                 state.scope_instructions.clearAndFree();
//             }
//         } else {
//             // pointer is nonzero, we're looping back
//             state.jump_level = 1;
//             // iterate back using the scope_instructions basket
//             if (state.scope_instructions.items.len == 0) {
//                 std.debug.print("zigby: internal error: could not jump back to a matching '[', there are no buffered instructions\n... state.scope_instructions: '{s}'\n", .{state.scope_instructions.items});
//                 return false;
//             }
//             var rev_idx = state.scope_instructions.items.len - 1;
//             while (state.jump_level > 0) {
//                 const prev_inst = &[_]u8{state.scope_instructions.items[rev_idx]};
//                 if (std.mem.eql(u8, prev_inst, "[")) {
//                     state.jump_level -= 1;
//                     break;
//                 } else if (std.mem.eql(u8, prev_inst, "]")) {
//                     state.jump_level += 1;
//                 }
//                 if (rev_idx == 0) {
//                     std.debug.print("zigby: internal error: could not jump back to a matching '[', ran out of buffered instructions\n... state.scope_instructions: '{s}'\n", .{state.scope_instructions.items});
//                     return false;
//                 } else {
//                     state.rewind_counter += 1;
//                     rev_idx -= 1;
//                 }
//             }
//         }
//     }
//     return true;
// }

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
        .instructions = std.MultiArrayList(Instruction){},
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

fn preprocess(stream: anytype, instructions: *std.MultiArrayList(Instruction), alloc: std.mem.Allocator) !void {
    var prog_line: u64 = 1;
    var prog_char: u64 = 0;

    var running_character: u8 = 0;
    var running_character_count: u32 = 0;
    
    var instruction_map = std.StringHashMap(SingleInstruction).init(std.heap.page_allocator);
    try instruction_map.put(">", .POINTER_RIGHT);
    try instruction_map.put("<", .POINTER_LEFT);
    try instruction_map.put("+", .CELL_INCREMENT);
    try instruction_map.put("-", .CELL_DECREMENT);
    try instruction_map.put(".", .OUTPUT);
    try instruction_map.put(",", .INPUT);
    try instruction_map.put("[", .JUMP_FORWARD);
    try instruction_map.put("]", .JUMP_BACKWARD);

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
