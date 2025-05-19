// zigby: a brainfuck interpreter in zig

const std = @import("std");

const ProgramState = struct {
    cells: [30000]u8 = [_]u8{0} ** 30000,
    cell_counter: u64 = 0,
    rewind_counter: u64 = 0,
    scope_depth: u64 = 0,
    scope_instructions: std.ArrayList(u8),
    scope_jump_past: bool = false,
    jump_level: u64 = 0,
};

pub fn interpret(_inst: u8, state: *ProgramState) bool {
    var inst = _inst;
    var inst_slice: []const u8 = &[_]u8{_inst};

    // are we jumping past a scope? if so, keep track of nestled scopes so we
    // can resume where we want to
    if (state.scope_jump_past) {
        if (std.mem.eql(u8, inst_slice, "[")) {
            state.jump_level += 1;
        } else if (std.mem.eql(u8, inst_slice, "]")) {
            state.jump_level -= 1;
            if (state.jump_level == 0) {
                state.scope_jump_past = false;
            }
        }

        return true;
    }

    // are we currently reinterpreting a scope? (is rewind_counter > 0?)
    // if so, override inst and inst_slice to the
    // (scope_instructions.len - rewind_counter)th index of scope_instructions
    // and use that as the current inst
    // (interpret the buffered scope until we exit it)
    if (state.rewind_counter > 0) {
        if ((state.rewind_counter > state.scope_instructions.items.len) and (state.scope_instructions.items.len >= (state.scope_instructions.items.len - state.rewind_counter))) {
            std.debug.print("zigby: internal error: attempting to rewind further ({}) than the buffered scope instructions ({})\n", .{ state.rewind_counter, state.scope_instructions.items.len });
            return false;
        }

        inst = state.scope_instructions.items[state.scope_instructions.items.len - state.rewind_counter];
        inst_slice = &[_]u8{inst};
        // std.debug.print("debug: rewinding to instruction: {c}\n", .{inst});

        state.rewind_counter -= 1;
    }
    // if we're not reinterpreting a scope, and the scope level is not 0, we
    // should be keeping track of the scope instructions in case we need to
    // rewind to them later
    else if (state.scope_depth != 0) {
        state.scope_instructions.append(inst) catch |err| {
            std.debug.print("zigby: internal error: failed to append instruction to scope instructions: {}\n", .{err});
            return false;
        };
    }

    // standard program execution

    // https://esolangs.org/wiki/Brainfuck
    // >   Move the pointer to the right
    if (std.mem.eql(u8, inst_slice, ">")) {
        if (state.cell_counter >= state.cells.len) {
            std.debug.print("error: pointer increment out of bounds (@ cell {})\n", .{state.cell_counter});
            return false;
        }
        state.cell_counter += 1;
    }

    // <   Move the pointer to the left
    if (std.mem.eql(u8, inst_slice, "<")) {
        if (state.cell_counter == 0) {
            std.debug.print("error: pointer decrement out of bounds (@ cell {})\n", .{state.cell_counter});
            return false;
        }
        state.cell_counter -= 1;
    }

    // +   Increment the memory cell at the pointer
    if (std.mem.eql(u8, inst_slice, "+")) {
        const result, _ = @addWithOverflow(state.cells[state.cell_counter], 1);
        state.cells[state.cell_counter] = result;
    }

    // -   Decrement the memory cell at the pointer
    if (std.mem.eql(u8, inst_slice, "-")) {
        const result, _ = @subWithOverflow(state.cells[state.cell_counter], 1);
        state.cells[state.cell_counter] = result;
    }

    // .   Output the character signified by the cell at the pointer
    if (std.mem.eql(u8, inst_slice, ".")) {
        std.io.getStdOut().writer().print("{c}", .{state.cells[state.cell_counter]}) catch |err| {
            std.debug.print("zigby: internal error: could not output cell {}: {s}\n", .{ state.cell_counter, @errorName(err) });
            return false;
        };
    }

    // ,   Input a character and store it in the cell at the pointer
    if (std.mem.eql(u8, inst_slice, ",")) {
        state.cells[state.cell_counter] = std.io.getStdIn().reader().readByte() catch |err| {
            std.debug.print("zigby: internal error: could not input cell {}: {s}\n", .{ state.cell_counter, @errorName(err) });
            return false;
        };
    }

    // [   Jump past the matching ']' if the cell at the pointer is 0
    if (std.mem.eql(u8, inst_slice, "[")) {
        if (state.cells[state.cell_counter] == 0) {
            // we're zero, we'll skip this current scope
            state.jump_level = 1;
            state.scope_jump_past = true;
        } else {
            // we're nonzero, we'll keep executing
            state.scope_depth += 1;
            state.scope_instructions.append(inst) catch |err| {
                std.debug.print("zigby: internal error: failed to append instruction to scope instructions: {}\n", .{err});
                return false;
            };
        }
    }

    // ]   Jump back to the matching '[' if the cell at the pointer is nonzero
    if (std.mem.eql(u8, inst_slice, "]")) {
        if (state.cells[state.cell_counter] == 0) {
            // we're zero, we're getting out
            state.scope_depth -= 1;

            // are we back at the top? if so, it's now safe to clear the
            // instruction basket
            if (state.scope_depth == 0) {
                state.scope_instructions.clearAndFree();
            }
        } else {
            // pointer is nonzero, we're looping back
            state.jump_level = 1;

            // iterate back using the scope_instructions basket
            if (state.scope_instructions.items.len == 0) {
                std.debug.print("zigby: internal error: could not jump back to a matching '[', there are no buffered instructions\n... state.scope_instructions: '{s}'\n", .{state.scope_instructions.items});
                return false;
            }

            var rev_idx = state.scope_instructions.items.len - 1;
            while (state.jump_level > 0) {
                const prev_inst = &[_]u8{state.scope_instructions.items[rev_idx]};

                if (std.mem.eql(u8, prev_inst, "[")) {
                    state.jump_level -= 1;
                    break;
                } else if (std.mem.eql(u8, prev_inst, "]")) {
                    state.jump_level += 1;
                }

                if (rev_idx == 0) {
                    std.debug.print("zigby: internal error: could not jump back to a matching '[', ran out of buffered instructions\n... state.scope_instructions: '{s}'\n", .{state.scope_instructions.items});
                    return false;
                } else {
                    state.rewind_counter += 1;
                    rev_idx -= 1;
                }
            }
        }
    }

    return true;
}

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

    // 3. init program state
    var dba = std.heap.DebugAllocator(.{}){};
    const dbg_alloc = dba.allocator();
    defer _ = dba.deinit();

    var state = ProgramState{
        .scope_instructions = std.ArrayList(u8).init(dbg_alloc),
    };
    defer state.scope_instructions.deinit();

    // 4. read file byte by byte and interpret
    var prog_line: u64 = 1;
    var prog_char: u64 = 0;
    while (true) {
        const current_char = stream.readByte() catch |err| {
            if (err == error.EndOfStream) {
                break;
            } else {
                std.debug.print("zigby: error: could not read byte: {s}\n", .{@errorName(err)});
                return;
            }
        };

        if (std.mem.eql(u8, &[_]u8{current_char}, "\n")) {
            prog_line += 1;
            prog_char = 0;
        }

        // std.debug.print("debug: main ({c})\n", .{current_char});
        const ok = interpret(current_char, &state);
        prog_char += 1;
        if (!ok) {
            std.debug.print("... note: execution stopped at line {}, character {}", .{ prog_line, prog_char });
            return;
        }

        while (state.rewind_counter > 0) {
            // std.debug.print("debug: rewind {} back from {}:{}\n", .{state.rewind_counter, prog_line, prog_char});
            const rok = interpret(0, &state);
            if (!rok) {
                std.debug.print("... note: execution stopped at line {}, character {}", .{ prog_line, prog_char });
                return;
            }
        }
    }
}
