"""
daycircle.cli: command line entry and interface for daycircle
-------------------------------------------------------------
by mark <mark@joshwel.co>

This is free and unencumbered software released into the public domain.

Anyone is free to copy, modify, publish, use, compile, sell, or
distribute this software, either in source code form or as a compiled
binary, for any purpose, commercial or non-commercial, and by any
means.

In jurisdictions that recognize copyright laws, the author or authors
of this software dedicate any and all copyright interest in the
software to the public domain. We make this dedication for the benefit
of the public at large and to the detriment of our heirs and
successors. We intend this dedication to be an overt act of
relinquishment in perpetuity of all present and future rights to this
software under copyright law.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR
OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
OTHER DEALINGS IN THE SOFTWARE.

For more information, please refer to <http://unlicense.org/>
"""

from argparse import ArgumentParser
from pathlib import Path
from sys import stderr
from typing import Generator, NamedTuple

from .analyser import analyse
from .grapher import graph
from .parser import DaycircleColour, DaycircleFileData, parse
from .utils import Result


class Behaviour(NamedTuple):
    """namedtuple representing daycircle behaviour"""

    targets: list[Path]
    colour_file: Path | None = None
    output: Path | None = None
    output_format: str = "svg"
    font_path: Path | None = None
    process_individually: bool = False


def handle_args() -> Behaviour:
    """handle command-line arguments and return a list of pathlib.Path targets"""
    parser = ArgumentParser(
        description="beautifully chart your average day over a period of time"
    )
    parser.add_argument(
        "targets",
        nargs="*",
        type=Path,
        help="the target daycircle plaintext file(s) to read from",
    )
    parser.add_argument(
        "-c",
        "--colour-file",
        type=Path,
        default=None,
        help="a target daycircle plaintext with colours keys to use",
    )
    parser.add_argument(
        "-o",
        "--output",
        type=Path,
        default=None,
        help="the output file or directory to write to",
    )
    parser.add_argument(
        "-t",
        "--output-type",
        type=str,
        choices=["png", "svg", "pdf"],
        default="svg",
        help="the output format to use, overrides any file extension given to --output",
    )
    parser.add_argument(
        "-f",
        "--font",
        type=Path,
        default=None,
        help="the font to use for the graph",
    )
    parser.add_argument(
        "-i", "--process-individually",
        action="store_true",
        # type=bool,
        default=False,
        help="process each target individually, instead of as a whole",
    )
    args = parser.parse_args()
    return Behaviour(
        targets=args.targets,
        colour_file=args.colour_file,
        output=args.output if isinstance(args.output, Path) else None,
        output_format=args.output_type,
        font_path=args.font
        if (isinstance(args.font, Path) and args.font.is_file())
        else None,
        process_individually=args.process_individually,
    )


def entry() -> None:
    """command-line entry point for daycircle"""

    # read files
    behaviour = handle_args()
    data: list[DaycircleFileData] = []
    event_colours: dict[str, DaycircleColour] = {}
    processed: int = 0

    def read_file(target: Path) -> None:
        nonlocal processed

        try:
            target_data = parse(
                target.read_text(encoding="utf-8"),
                filename=target.name,
            )

        except Exception as e:
            print(f"warn: failed to read '{target}': {e}, skipping", file=stderr)

        else:
            if not target_data:
                print("warn:", target_data.cry(string=True), file=stderr)
                return

            data.append(target_data.get())
            processed += 1

    def walk_targets() -> Generator[Path, None, None]:
        for target in behaviour.targets:
            if not target.exists():
                print(f"warn: '{target}' does not exist, skipping", file=stderr)
                continue

            if target.is_file():
                yield target
                continue

            elif target.is_dir():
                print(f"info: recursively entering '{target}'", file=stderr)
                for file in target.rglob("*"):
                    yield file
                continue

            print(f"warn: skipped '{target}' (not a file or directory)", file=stderr)

    for file in walk_targets():
        read_file(file)

    if behaviour.colour_file is not None:
        if not (behaviour.colour_file.exists() and behaviour.colour_file.is_file()):
            print(
                f"error: colour file '{behaviour.colour_file}' is not a valid file",
                file=stderr,
            )
            return

        colour_file_data = parse(
            behaviour.colour_file.read_text(encoding="utf-8"), is_colour_file=True
        ).get()
        event_colours = colour_file_data.event_colours
        print(
            f"info: using {len(event_colours)} colours from '{behaviour.colour_file}'",
            file=stderr,
        )

    if (event_colours == {}) and (len(data) > 0 and data[0].event_colours != {}):
        event_colours = data[0].event_colours
        print(
            f"info: using {len(event_colours)} colours from first target ({data[0].day})",
            file=stderr,
        )

    print(f"info: successfully read {processed} file(s)", file=stderr)

    # analyse data
    for graph_data in analyse(targets=data, process_individually=behaviour.process_individually):
        # plot data
        graph_image = graph(
            data=graph_data,
            event_colours=event_colours,
            font_path=behaviour.font_path,
            format=behaviour.output_format,
        ).get()

        # write output
        if behaviour.output is not None:
            if behaviour.output.is_dir():
                behaviour.output.mkdir(parents=True, exist_ok=True)
            else:
                behaviour.output.parent.mkdir(parents=True, exist_ok=True)

        with open(
            output_filename := graph_data.to_filename(
                name_override=behaviour.output,
                file_type=behaviour.output_format,
            ),
            mode="wb",
        ) as output_file:
            print(f"info: writing graph to {output_filename}...", end="", file=stderr)
            output_file.write(graph_image.getbuffer())
            print(" done", file=stderr)
