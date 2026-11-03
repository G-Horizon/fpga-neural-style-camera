// board/ddr3/d64_blk_beats_width.vh -- how many bits a block's beat counter needs.
//
// **A second file because a 2001 module cannot see a file-scope function**, and
// because a backtick-define is too late for the parameter defaults that need it: the
// macros in `d64_blk_beats.vh` are included *before* the module (the
// derivation is a parameter default), and this function is included
// *inside* the module body (the counter's width is a declaration).
//
// It is not `clog2(n)`: the block's beat counter reaches `BLK_BEATS` itself on
// the beat before its completion fires -- the count and the comparison are both
// the block's -- so `clog2` would truncate the very value the test is for.  And
// it never returns 0: a zero-width register is not a register.

function integer d64_beats_w;
    input integer n;
    integer i, w;
    begin
        w = 1;
        for (i = n; i > 1; i = i >> 1) w = w + 1;
        d64_beats_w = w;
    end
endfunction
