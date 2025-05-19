// [zigby -- an okay brainfuck interpreter in zig
// (c) 2025 mark joshwel <mark@joshwel.co>
// Zero-Clause BSD Licence
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

/// a packed enum spanning three bits, giving 2^3 = 8 possible values,
/// which nicely correspond to the amount of brainfuck instructions
/// (used in InstructionPackage)
const Instruction = enum(u3) {
    OUTPUT,
    INPUT,
    POINTER_LEFT,
    POINTER_RIGHT,
    CELL_INCREMENT,
    CELL_DECREMENT,
    JUMP_FORWARD,
    JUMP_BACKWARD,
};

/// a compile time generated lookup table using a 256-wide boolean array,
/// allows for u8 (0-255) characters to be used as numerical indexes and
/// return a false/true for fast look up compared to my older usage of
/// `std.mem.indexOfScalar`
const LOOKUP_INSTRUCTION_CHAR_TO_BOOL = blk: {
    var lookup = [_]bool{false} ** 256;
    lookup['<'] = true;
    lookup['>'] = true;
    lookup['+'] = true;
    lookup['-'] = true;
    lookup['.'] = true;
    lookup[','] = true;
    lookup['['] = true;
    lookup[']'] = true;
    break :blk lookup;
};

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

/// file buffer size, was considering 4k vs 8k vs 16k,
/// but 16k chosen mainly for bad apple
const FILE_BUFFER_SIZE = 16384;

/// a packed struct spanning 32 bits that store 7 instructions,
/// an instruction count (`.instruction_count=3` would mean
/// `.instruction_{1...3}` are to be executed), and a repitition count.
///
/// considering an ascii/unicode character is 8 bits wide,
/// the worst case scenario sees 7 instructions packed into a 32-bit
/// `InstructionPackage`, 75% more insts in a u32, compared to 4 insts as u8s.
///
/// the best case scenario is a repeating pattern of 7 instructions up to 256
/// times, packing 1792 instructions into one `InstructionPackage`, 448% more
/// than naive the u8 representation
/// (but this will rarely ever been seen, probably)
const InstructionPackage = packed struct(u32) {
    instruction_count: u3,
    instruction_1: Instruction,
    instruction_2: Instruction,
    instruction_3: Instruction,
    instruction_4: Instruction,
    instruction_5: Instruction,
    instruction_6: Instruction,
    instruction_7: Instruction,
    repeat_count: u8 = 0,
};

/// an abstracted program iterator that:
/// - opens a file and holds a bufferedreader set to 16k (2048 u8 characters)
/// - reads from the next character
/// - keeps track of where in the program are we
/// - keeps track of scopes
/// - allows for virtual jumping as long as the position
/// - skips initial immediate scopes that hold comments
pub fn AbstractedProgramIterator(comptime UnbufferedReaderType: type) type {
    return struct {
        const GenericisedAbsProgIter = @This();

        alloc: std.mem.Allocator,
        _reader: std.io.BufferedReader(FILE_BUFFER_SIZE, UnbufferedReaderType),
        reader: std.io.BufferedReader(FILE_BUFFER_SIZE, UnbufferedReaderType).Reader,

        /// raw buffer for reading into
        buffer: [64]u8,
        /// index of where are we in the buffer, if its any higher than 63,
        /// treat the buffer as fresh/exhausted and read into it
        buffer_index: u8 = 255,
        /// the max character the buffer is filled to, is set when the buffer
        /// has lesser actual data than length, so when buffer_index is equal
        /// to the max buffer index, then we're at the last character
        max_buffer_index: usize = 64,

        /// the virtual position, not including skipped instructions/characters
        position: u64 = 0,
        /// the actual position
        position_actual: u64 = 0,
        /// the current line number based on past-seen newlines, used for error printing
        position_as_line_number: u64 = 1,
        /// the current column number based on past-seen newlines, used for error printing
        position_as_column_number: u64 = 1,

        /// what current depth level are we in, used to clear the
        /// scope_tracking_buffer and relevant scope hashmaps when
        /// we're back up at a global scope
        scope_depth: u64 = 0,
        /// hash map for keeping track of the instructions of relevant scopes
        scope_start_position_to_instructions_map: std.AutoHashMap(u64, std.ArrayList(InstructionPackage)),
        /// hash map for keeping track of the brace positions of relevant scopes
        scope_end_position_to_start_position_map: std.AutoHashMap(u64, u64),

        /// tracking variable for when in a scope,
        /// holds the current scopes' instructions
        scope_tracking_buffer: std.ArrayList(InstructionPackage),
        /// tracking variable for when in a scope, true if we're skipping and not
        /// executing in the current scope
        scope_tracking_skip_current: bool = false,
        /// tracking variable for when in a scope, tracks how many child scopes
        /// we've encountered
        scope_tracking_depth: u64 = 0,

        /// initialisation function
        pub fn init(unbuffered_reader: UnbufferedReaderType, alloc: std.mem.Allocator) GenericisedAbsProgIter {
            var reader = std.io.BufferedReader(FILE_BUFFER_SIZE, UnbufferedReaderType){ .unbuffered_reader = unbuffered_reader };
            return GenericisedAbsProgIter{
                .alloc = alloc,
                ._reader = reader,
                .reader = reader.reader(),
                .buffer = [_]u8{0} ** 64,
                .scope_start_position_to_instructions_map = .init(alloc),
                .scope_end_position_to_start_position_map = .init(alloc),
                .scope_tracking_buffer = .init(alloc),
            };
        }

        /// deinitialisation function, deinit-s everything inside the struct
        /// that would need to be deinit-ed at defer time
        pub fn deinit(self: *GenericisedAbsProgIter) void {
            self.scope_start_position_to_instructions_map.deinit();
            self.scope_end_position_to_start_position_map.deinit();
            self.scope_tracking_buffer.deinit();
        }

        /// helper function to read a character from the buffer
        ///
        /// returns !?u8:
        /// - ! (erroneous) -> something internal fucked up
        /// - ? (null) -> reached end of buffer
        /// - u8 -> the read character
        ///
        /// note: the buffer_index is NOT incremented.
        pub inline fn readCharAtBufferIndex(self: *GenericisedAbsProgIter) !?u8 {
            if (self.buffer_index >= self.buffer.len) {
                self.max_buffer_index = try self.reader.readAtLeast(&self.buffer, self.buffer.len);
                self.buffer_index = 0;
            }

            if (self.buffer_index == self.max_buffer_index) {
                return null;
            }

            return self.buffer[self.buffer_index];
        }

        /// prime the iterator by doing a pre-pass of any characters that are
        /// not instructions, or directly-infront scopes (usually used for
        /// comments)
        ///
        /// returns a !?InterpretationResult:
        /// - erroneous -> something internal fucked up
        /// - InterpretationResult -> check .ok, else use .err for error
        ///   description
        /// - null -> reached end of readable file/EOF
        ///
        /// this function will not consume the instruction it stops on, and
        /// will be directly accessible via `.buffer[.buffer_index]` post-call
        ///
        /// this does not check if has been called previously and thus can be
        /// used for both a pre-execution pre-pass and as a scope-skipping
        /// helper during execution
        ///
        /// access .position_actual before and/or after to find out how many
        /// characters were skipped.
        pub inline fn scopeSkip(self: *GenericisedAbsProgIter) !?InterpretationResult {
            var are_we_skipping: bool = false;
            var current_depth_level: u64 = 0;

            while (true) {
                // variable to control whether or not to advance counters,
                // used when we actually want to start executing at this current
                // character, and to not 'consume' it
                var run_defer: bool = true;

                // read a character
                const current_char = try self.readCharAtBufferIndex() orelse {
                    return null;
                };

                // advance position counters
                defer {
                    if (run_defer) {
                        self.position_actual += 1;
                        if (current_char == '\n') {
                            self.position_as_line_number += 1;
                            self.position_as_column_number = 0;
                        }
                        self.position_as_column_number += 1;
                    }
                    self.buffer_index += 1;
                }

                // skip if it isnt an instruction
                if (!LOOKUP_INSTRUCTION_CHAR_TO_BOOL[current_char]) {
                    // self.buffer_index += 1;
                    continue;
                }

                if (are_we_skipping) {
                    switch (current_char) {
                        // handle encountering scope start and ends
                        '[' => {
                            current_depth_level += 1;
                        },
                        ']' => {
                            current_depth_level -= 1;

                            if (current_depth_level == 0) {
                                are_we_skipping = false;
                            }
                        },
                        else => {
                            // self.buffer_index += 1;
                            // continue;
                        },
                    }
                } else {
                    switch (current_char) {
                        // but we encountered one, so lets its time to skip
                        '[' => {
                            current_depth_level = 1;
                            are_we_skipping = true;
                        },
                        ']' => {
                            return InterpretationResult{
                                .ok = false,
                                .err = "encountered a ']' with no previous matching '['",
                            };
                        },
                        // we're not skipping, and we have encountered a valid instruction,
                        // its time to actually start executing
                        else => {
                            run_defer = false;

                            self.position += 1;
                            return InterpretationResult{
                                .ok = true,
                            };
                        },
                    }
                }
            }

            return InterpretationResult{
                .ok = true,
            };
        }

        /// iteration function: collects the current buffer into the first
        /// InstructionPackage it can output
        ///
        /// returns a !?InstructionPackage:
        /// - ! (erroneous) -> something internal fucked up
        /// - ? (null) -> reached end of file/buffer
        /// - InstructionPackage -> an instruction package
        pub inline fn next(self: *GenericisedAbsProgIter) !?InstructionPackage {
            var remaining_characters = self.max_buffer_index - self.buffer_index;
            var instpkg = InstructionPackage{
                .instruction_1 = Instruction.OUTPUT,
                .instruction_2 = Instruction.OUTPUT,
                .instruction_3 = Instruction.OUTPUT,
                .instruction_4 = Instruction.OUTPUT,
                .instruction_5 = Instruction.OUTPUT,
                .instruction_6 = Instruction.OUTPUT,
                .instruction_7 = Instruction.OUTPUT,
                .instruction_count = @intCast(@min(remaining_characters, 7)),
            };
            
            if (remaining_characters == 0) {
            }

            return instpkg;
        }
    };
}

/// a struct representing the current interpreter state,
/// for the interpret function to take in
pub fn InterpreterState(comptime UnbufferedReaderType: type) type {
    return struct {
        const GenericisedInterpreterState = @This();

        /// the abstract program iterator
        program: AbstractedProgramIterator(UnbufferedReaderType),
        /// the memory cells, an array of u8.
        /// resize to 30,000 before program execution
        data_cells: std.ArrayList(u8),
        /// the data cell pointer
        data_pointer: u64 = 0,

        /// helper function to deinit everything inside the struct that would
        /// need to be deinit-ed at defer time
        pub fn deinit(self: *GenericisedInterpreterState) void {
            defer self.program.deinit();
            defer self.data_cells.deinit();
        }
    };
}

const InterpretationResult = struct {
    /// whether or not interpretation was succesful
    ok: bool,
    /// error string
    err: ?[]const u8 = null,
};

/// interpretation function holding the core runtime loop
pub inline fn interpret(comptime T: type, state: *InterpreterState(T)) !InterpretationResult {
    while (try state.program.next()) |instpkg| {
        // DEBUG
        var lookup = [_]u8{' '} ** 8;
        lookup[@intFromEnum(Instruction.POINTER_LEFT)] = '<';
        lookup[@intFromEnum(Instruction.POINTER_RIGHT)] = '>';
        lookup[@intFromEnum(Instruction.CELL_INCREMENT)] = '+';
        lookup[@intFromEnum(Instruction.CELL_DECREMENT)] = '-';
        lookup[@intFromEnum(Instruction.JUMP_FORWARD)] = '[';
        lookup[@intFromEnum(Instruction.JUMP_BACKWARD)] = ']';
        lookup[@intFromEnum(Instruction.INPUT)] = '.';
        lookup[@intFromEnum(Instruction.OUTPUT)] = ',';
        std.debug.print("zigby(debug): position={} -> InstructionPackage( '{c}', '{c}', '{c}', '{c}', '{c}', '{c}', '{c}' )\n", .{ state.program.position, lookup[@intFromEnum(instpkg.instruction_1)], lookup[@intFromEnum(instpkg.instruction_2)], lookup[@intFromEnum(instpkg.instruction_3)], lookup[@intFromEnum(instpkg.instruction_4)], lookup[@intFromEnum(instpkg.instruction_5)], lookup[@intFromEnum(instpkg.instruction_6)], lookup[@intFromEnum(instpkg.instruction_7)] });
    }
    else {
        return .{ .ok = false };
    }

    
    return .{ .ok = true };
}

/// main function: process target file arg, read and pass to interpreter
pub fn main() void {
    var arena = std.heap.ArenaAllocator.init(std.heap.page_allocator);
    defer arena.deinit();
    const arena_alloc = arena.allocator();

    // 1. get arguments
    var args_iter = std.process.argsWithAllocator(arena_alloc) catch |err| {
        std.debug.print("zigby: internal error: {s}\n", .{@errorName(err)});
        std.process.exit(10);
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

    if (args_counter <= 1) {
        std.debug.print("zigby: error: no target file specified\n", .{});
        std.process.exit(11);
    }

    if (target == null) {
        std.debug.print("zigby: internal error: target file string is null\n", .{});
        std.process.exit(12);
    }

    // 2. open file
    const file = std.fs.cwd().openFile(target.?, std.fs.File.OpenFlags{}) catch |err| {
        std.debug.print("zigby: error: could not open file '{s}' ({s})\n", .{ target.?, @errorName(err) });
        std.process.exit(20);
    };
    const file_reader = file.reader();
    defer file.close();

    // 3. init stuff
    var runtime_alloc_mgr = std.heap.DebugAllocator(.{}){};
    const runtime_alloc = runtime_alloc_mgr.allocator();
    defer _ = runtime_alloc_mgr.deinit();

    // generics will always be my favourite thing and the end of me
    var state = InterpreterState(@TypeOf(file_reader)){
        .program = AbstractedProgramIterator(@TypeOf(file_reader)).init(file_reader, runtime_alloc),
        .data_cells = .init(runtime_alloc),
    };
    defer state.deinit();

    state.data_cells.resize(30000) catch |err| {
        std.debug.print("zigby: internal error: could not resize data cell/memory tape array to 30,000: {s}\n", .{@errorName(err)});
        std.process.exit(30);
    };
    @memset(state.data_cells.items, 0);

    // 4. scopeskip prepass / interpreter priming
    const ss_result = state.program.scopeSkip() catch |err| {
        std.debug.print("zigby: internal error: could not do a scopeskip prepass ({s})\n", .{@errorName(err)});
        std.process.exit(40);
    };
    if (ss_result == null) {
        std.debug.print("zigby: warning: not doing anything, no valid instructions to execute (scopeskip prepass exhausted the program length)\n", .{});
        std.process.exit(41);
    }
    if (!ss_result.?.ok) {
        if (ss_result.?.err == null) {
            std.debug.print("zigby: error: ???\n", .{});
        } else {
            std.debug.print("zigby: error: {s}\n", .{ss_result.?.err.?});
        }
        std.debug.print("... note: prepass interpretation stopped at line {}, column {}\n", .{ state.program.position_as_column_number, state.program.position_as_line_number });
        std.process.exit(1);
    }

    // program execution
    const result = interpret(@TypeOf(file_reader), &state) catch |err| {
        std.debug.print(
            \\zigby: internal error: could not interpret program ({s})
            \\... note: execution stopped at line {}, column {}
            \\
        , .{ @errorName(err), state.program.position_as_column_number, state.program.position_as_line_number });
        std.process.exit(255);
    };
    if (!result.ok) {
        if (result.err == null) {
            std.debug.print("zigby: error: ???\n", .{});
        } else {
            std.debug.print("zigby: error: {s}\n", .{result.err.?});
        }
        std.debug.print("... note: interpretation stopped at line {}, column {}\n", .{ state.program.position_as_column_number, state.program.position_as_line_number });
        std.process.exit(1);
    }
}
