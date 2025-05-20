# sota staircase SideStepper

a fast enough .gitignore-respecting large file finder

made for me to find large files in my unity game repositories that wouldn't
fit github's 100mb limit, for when i'd push my repositories to github for
schoolwork submission

rewritten from [python](https://forge.joshwel.co/mark/sota/src/branch/main/sidestepper.py)
to rust, as a reason to learn rust

**this is brain made software**: large language-based code generation has not
been extensively used here. but i'd be lying if i said i didn't ask chatgpt if
there was a better way to check a boolean result lol

## quickstart

### installing a binary

- [Windows](https://forge.joshwel.co/mark/sidestepper/releases/download/latest/sidestepper-windows-x86_64.exe)
- [Linux (static binary, musl)](https://forge.joshwel.co/mark/sidestepper/releases/download/latest/sidestepper-linux-x86_64)
- [macOS universal](https://forge.joshwel.co/mark/sidestepper/releases/download/latest/sidestepper-macos-universal)
- [macOS x86_64](https://forge.joshwel.co/mark/sidestepper/releases/download/latest/sidestepper-macos-x86_64)
- [macOS aarch64](https://forge.joshwel.co/mark/sidestepper/releases/download/latest/sidestepper-macos-aarch64)

(also available in the 'releases' tab wherever this repository is situated in)

### build it yourself

1. [get rust and cargo](https://doc.rust-lang.org/cargo/getting-started/installation.html#install-rust-and-cargo)
2. run `cargo build release`

**nix users, rejoice:** `nix run github:markjoshwel/sidestepper` or
`git+https://forge.joshwel.co/mark/sidestepper.git` (or nix build it, or add it
to your profile, or your system configuration, whatever)

### running it

```text
./sidestepper
```

or on windows,

```text
./sidestepper.exe
```

it'll find for a `.git` directory in the current or parent directories, if you
want to use this not in the context i usually use this for (which is for git
repositories), pass in `--search-here` to treat the current working directory
as the 'repository root'

it'll then make a `.sotaignore` file that i use in my other tooling scripts,
but if you want it to output external-tool-friendly output to stdout, pass in
`--plumbing` for it to output encountered large files, line-by-line, to stdout

## historical changes

- v5 (i3/a6) - fix for a silly bug (not properly checking if a .sotaignore entry already exists before writing it)
- v5 (i3/a5) - 3rd implementation, rewritten in rust lol (no longer using iod-ttt,
  just piggybacking off [ignore](https://crates.io/crates/ignore)'s WalkBuilder)
- v4 (i2/a4) - optimised single iod-ttt
- v3 (i2/a3) - faster matching by remembering ignored directories (ignore on demand, 'iod')
- v2 (i2/a2) - 2nd implementation, corrected ignored directory matching (named 'trytrytry', 'ttt')
- v1 (i1/a1) - original python script, still embedded within ReStepper

## licence

with all my heart, copyright (c) 2025 mark joshwel

the sota staircase SideStepper is permissively licenced, not needing
attribution, under the [0BSD licence](LICENCE). go ham.
