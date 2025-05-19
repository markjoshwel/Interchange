# zigby: an okay brainfuck interpreter written in zig

*okay meaning:

1. reasonably fast
    - direct execution model, no pre-processing
    - relevant scope braces are tracked for jump optimisations
    - skips any immediate `[]` scopes
    - special code paths for common brainfuck patterns like `[+]`, etc

2. reasonably low in memory usage
    - only relevant scopes are kept in-memory
    - scopes are naively run length encoded and/or bitpacked

3. compliant (i think)
    - growable memory tape size of at least 30k, zero-initialised
    - errors on data pointers moving past tape bounds
    - errors on mismatched brackets
    - follow's [daniel cristofani's epistle to the implementors](https://brainfuck.org/epistle.html)
      - end of line is `10` (ascii line feed)
      - **note:** EOF on `,` (input command) will leave the cell unchanged
      - **another note:** the `#` and `!` unofficial commands are not implemented
    - runs wikipedia's and brainfuck.org's implementation-testing programs and normal example programs

why i wrote this the way i did: to run OpenSauce04's 200mb
[bad apple](https://github.com/OpenSauce04/BadAppleBF) brainfuck file without
having to wait a long time for preprocessing whilst still being fast enough

## usage

1. get a copy of [zig](https://ziglang.org/download/) and compile it with `zig build`
2. run it with `zig-out/bin/zigby[.exe] file.bf` (file extension does not matter)
3. optionally run tests with `python tests/test.py`

## licence

zigby is permissively "i do not care" licenced with the zero-clause bsd licence, go ham
