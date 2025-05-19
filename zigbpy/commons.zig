const std = @import("std");

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

pub fn output(s: *State(), by: usize) void {
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
