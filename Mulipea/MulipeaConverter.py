# /// script
# requires-python = ">=3.10"
# dependencies = [
#     "mutagen",
#     "pillow",
#     "rich",
# ]
# ///
#
# Copyright (c) 2025 mark joshwel <mark@joshwel.co>
# Zero-Clause BSD Licence
#
# Permission to use, copy, modify, and/or distribute this software for
# any purpose with or without fee is hereby granted.
#
# THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
# WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
# OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
# FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
# DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
# AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
# OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

from io import BytesIO
from multiprocessing import Manager
from multiprocessing.managers import DictProxy
from os import get_terminal_size, remove, stat, terminal_size
from pathlib import Path
from shutil import copy, move
from subprocess import PIPE, CompletedProcess, run
from sys import argv
from tempfile import TemporaryDirectory
from typing import Any, Callable, NamedTuple, TextIO

import mutagen  # type: ignore
from mutagen.flac import FLAC  # type: ignore
from mutagen.id3 import MVIN  # type: ignore
from mutagen.id3 import TBPM  # type: ignore
from mutagen.id3 import TCMP  # type: ignore
from mutagen.id3 import TPOS  # type: ignore
from mutagen.id3 import TRCK  # type: ignore
from mutagen.id3 import USLT  # type: ignore
from mutagen.id3 import TextFrame  # type: ignore
from mutagen.mp3 import MP3  # type: ignore
from mutagen.mp4 import MP4, AtomDataType, MP4Cover  # type: ignore
from mutagen.oggvorbis import OggVorbis  # type: ignore
from PIL import Image  # type: ignore
from rich.progress import Progress  # type: ignore

__version__: str = "0.2.15"
DIR_MULIPEA: Path = Path(__file__).parent.joinpath("mulipea")
DIR_MUSIC: Path = Path(__file__).parent.joinpath("music")
DIR_LOGS: Path = DIR_MULIPEA.joinpath(".logs")

FORCE_DEBUG: bool = False
MAX_RETRY_ATTEMPTS: int = 3
PROGRESS_BAR_SPACE: float = 0.50
PROCESSOR_SATURATION: float = 1.00
MAX_LARGEST_COVER_DIMENSION: int = 500
SWAP_ARTIST_AND_ALBUM_ARTIST: bool = False
SUPPORTED_FILE_TYPES: list[str] = [".m4a", ".mp3", ".ogg", ".flac"]
TAGS_MINIMUM: tuple[str, ...] = (
    # https://mutagen.readthedocs.io/en/latest/api/mp4.html
    # only have tags for music, no podcasts/audiobooks/tv shows
    "\xa9alb",  # album
    "\xa9ART",  # artist
    "\xa9nam",  # title
    "trkn",  # track number
)
TAGS_TO_KEEP: dict[str, type] = {
    # https://mutagen.readthedocs.io/en/latest/api/mp4.html
    # only have tags for music, no podcasts/audiobooks/tv shows
    "\xa9alb": str,  # album
    "\xa9ART": str,  # artist
    "\xa9nam": str,  # title
    "trkn": int,  # track number (tuple of, signalled by non-0 index in TAGS_MAP_*)
    # 'extra' tags
    "\xa9cmt": str,  # comment
    "\xa9day": str,  # year
    "\xa9gen": str,  # genre
    "\xa9lyr": str,  # lyrics
    "\xa9mvc": int,  # movement count
    "\xa9mvi": int,  # movement index
    "\xa9mvn": str,  # movement
    "\xa9wrk": int,  # work/movement (numeric)
    "\xa9wrt": str,  # composer
    "\xa9too": str,  # encoder
    "aART": str,  # album artist
    "covr": str,  # cover art
    "cpil": bool,  # is a compilation?
    "cprt": str,  # copyright
    "disk": int,  # disc number (tuple of, signalled by non-0 index in TAGS_MAP_*)
    "pgap": bool,  # is part of a gapless album?
    "purd": str,  # purchase date
    "soaa": str,  # album artist sort order
    "soal": str,  # album sort order
    "soar": str,  # artist sort order
    "sonm": str,  # title sort order
    "soco": str,  # composer sort order
    "tmpo": int,  # bpm
}

# from: (to, index <0 is replace, 1 is first, ...>)
TAGS_MAP_VORBIS_TO_MP4: dict[str, tuple[str, int]] = {
    "album": ("\xa9alb", 0),  # album
    "artist": ("\xa9ART", 0),  # artist
    "title": ("\xa9nam", 0),  # title
    "tracknumber": ("trkn", 1),  # track number
    # 'extra' tags
    "tracktotal": ("trkn", 2),  # total tracks
    "totaltracks": ("trkn", 2),  # total tracks
    "comment": ("\xa9cmt", 0),  # comment
    "date": ("\xa9day", 0),  # year
    "genre": ("\xa9gen", 0),  # genre
    "lyrics": ("\xa9lyr", 0),  # lyrics
    "unsyncedlyrics": ("\xa9lyr", 0),  # lyrics
    "composer": ("\xa9wrt", 0),  # composer
    "encoder": ("\xa9too", 0),  # encoder
    "albumartist": ("aART", 0),  # album artist
    "compilation": ("cpil", 0),  # is a compilation?
    "copyright": ("cprt", 0),  # copyright
    "discnumber": ("disk", 1),  # disc number
    "disctotal": ("disk", 2),  # total discs
    "totaldiscs": ("disk", 2),  # total discs
    "gapless": ("pgap", 0),  # is part of a gapless album? (probably bullshit)
    "purchase_date": ("purd", 0),  # purchase date (probably bullshit)
    "albumartistsort": ("soaa", 0),  # album artist sort order
    "albumsort": ("soal", 0),  # album sort order
    "artistsort": ("soar", 0),  # artist sort order
    "titlesort": ("sonm", 0),  # title sort order
    "composersort": ("soco", 0),  # composer sort order
    "bpm": ("tmpo", 0),  # bpm
}
TAGS_MAP_ID3_TO_MP4: dict[str, tuple[str, int]] = {
    # these are all mutagen.id3.TextFrames unless otherwise specified
    # bare minimum
    "TALB": ("\xa9alb", 0),  # album
    "TPE1": ("\xa9ART", 0),  # artist
    "TIT2": ("\xa9nam", 0),  # title
    "TRCK": ("trkn", 1),  # track number (mutagen.id3.NumericPartTextFrame)
    # extras
    "TXXX:TRACKTOTAL": ("trkn", 2),  # total tracks
    "COMM": ("\xa9cmt", 0),  # comment
    "TDRC": ("\xa9day", 0),  # recording time -> year (mutagen.id3.TimeStampTextFrame)
    "TCON": ("\xa9gen", 0),  # genre
    "MVIN": ("\xa9mvc", 0),  # movement count (mutagen.id3.NumericPartTextFrame)
    "MVNM": ("\xa9mvn", 0),  # movement
    "TCOM": ("\xa9wrt", 0),  # composer
    "TENC": ("\xa9too", 0),  # encoder
    "TPE2": ("aART", 0),  # band -> album artist
    "TCMP": ("cpil", 0),  # is a compilation? (mutagen.id3.NumericTextFrame)
    "TCOP": ("cprt", 0),  # copyright
    "TPOS": (
        "disk",
        1,
    ),  # part of set -> disc number (mutagen.id3.NumericPartTextFrame)
    "TSO2": ("soaa", 0),  # album artist sort order
    "TSOA": ("soal", 0),  # album sort order
    "TSOP": ("soar", 0),  # performer -> artist sort order
    "TSOT": ("sonm", 0),  # title sort order
    "TSOC": ("soco", 0),  # composer sort order
    "TBPM": ("tmpo", 0),  # bpm (mutagen.id3.NumericTextFrame)
    "TXXX:DISCTOTAL": ("disk", 2),  # total discs
    "TXXX:LYRICS": ("\xa9lyr", 0),  # lyrics
    "USLT": ("\xa9lyr", 0),  # lyrics  (mutagen.id3.USLT <- mutagen.id3.Frame)
}


class ProgressUpdate(NamedTuple):
    total: float | None = None
    completed: float | None = None
    advance: float | None = None
    description: str | None = None
    visible: bool | None = None
    refresh: bool = False
    remove: bool = False


def _dprint(*args, **kwargs):
    if ("--debug" in argv) or FORCE_DEBUG:
        print(*args, **kwargs)


# noinspection PyBroadException
def _log(path: Path | TextIO, text: str) -> None:
    try:
        _dprint(text)
        if isinstance(path, Path):
            path.parent.mkdir(parents=True, exist_ok=True)
            with path.open("a+", encoding="utf-8") as file:
                file.write(text + "\n")
                # file.flush()
        else:
            print(text, file=path, flush=True)
    except Exception:
        pass


def _helper_clean_up_lyrics(lyrics: str) -> str:
    """
    either the lyrics is .txt or .lrc: if its .txt, we dont do anything,
    but if its a .lrc i'll start with [??:??.??] or something similar we remove that
    """
    new_lyrics: list[str] = []
    for idx, line in enumerate(lyrics.strip().splitlines()):
        if not line.startswith("["):
            new_lyrics.append(line.strip())
            continue
        if "]" not in line:
            new_lyrics.append(line.strip())
            continue

        # remove all ":" and "." characters inbeteen the brackets
        # then see if its all digits
        # if it is, then its a timestamp and we should remove it

        bracketed_portion: str = line[line.index("[") + 1 : line.index("]")]
        if all(
            [
                (c in "0123456789")
                for c in bracketed_portion.replace(":", "").replace(".", "")
            ]
        ):
            new_lyrics.append(line[len(bracketed_portion) + 2 :].strip())
        else:
            new_lyrics.append(line.strip())

    return "\n".join(new_lyrics).strip()


def _helper_make_progress_task_text(left: str, right: str) -> str:
    fallback: str = right
    # match (left, right):
    #     case ("", ""):
    #         return ""
    #     case ("", _):
    #         fallback = right
    #     case (_, ""):
    #         fallback = left
    #     case _:
    #         fallback = f"{left}: {right}"

    try:
        # <-term_size---------------------------->
        # ......................... <-p_b_s------>
        # <-free_space------------> ..............
        # left :              right [progress bar]

        # prioritise showing the righthand text

        term_size: terminal_size = get_terminal_size()
        progress_bar_space = int(term_size.columns * 0.50)
        free_space: int = term_size.columns - progress_bar_space
        left_space: int = free_space - 1 - len(right)

        if len(left) > left_space:
            # aaaaabbbbbccccc -> aaa…ccc
            left = f"{left[: (left_space - 1) // 2]}…{left[-(((left_space - 1) // 2) + ((left_space - 1) % 2)) :]}"

        right = f"{right}".rjust(free_space - len(left))
        return f"{left}:{right}"

    except Exception:
        return fallback


def _helper_move(
    origin: Path,
    destination: Path,
    logfile_io: TextIO,
) -> bool:
    try:
        move(origin, destination)
        return True
    except Exception as e:
        if logfile_io is not None:
            print(
                f"_helper_move: error: failed to move '{origin}' "
                f"to '{destination}' ({e.__class__.__name__}: {e})",
                file=logfile_io,
                flush=True,
            )
        return False


def _helper_copy(
    origin: Path,
    destination: Path,
    logfile_io: TextIO,
) -> bool:
    try:
        copy(origin, destination)
        return True if origin.exists() else False
    except Exception as e:
        if logfile_io is not None:
            print(
                f"[mulipea] _helper_copy: error: failed to copy '{origin}' "
                f"to '{destination}' ({e.__class__.__name__}: {e})",
                file=logfile_io,
                flush=True,
            )
        return False


# noinspection PyBroadException
def _helper_update_progress(
    description: str,
    progress_dict: "DictProxy[int, ProgressUpdate] | None" = None,
    progress_task_id: int | None = None,
    completed: int | float | None = None,
) -> None:
    try:
        if (progress_dict is not None) and (progress_task_id is not None):
            progress_dict[progress_task_id] = ProgressUpdate(
                description=description,
                completed=completed,
            )
    except Exception:
        pass


def _mulipea_get_m4a_codec(target: Path, logfile_io: TextIO) -> str:
    """runs ffprobe, should return 'aac' or 'alac' - handle erroneously if returned anything else"""
    cp: CompletedProcess[bytes] = run(
        [
            "ffprobe",
            "-v",
            "error",
            "-select_streams",
            "a:0",
            "-show_entries",
            "stream=codec_name",
            "-of",
            "default=noprint_wrappers=1:nokey=1",
            target,
        ],
        stderr=logfile_io,
        stdout=PIPE,
    )
    if cp.returncode != 0:
        print(
            f"[mulipea] _mulipea_get_m4a_codec: error: failed to probe '{target}'",
            file=logfile_io,
            flush=True,
        )
        return ""
    return cp.stdout.decode().strip()


# noinspection PyShadowingBuiltins
def _mulipea_common_prehandle(input: Path) -> tuple[Path, Path]:
    relative_input: Path = input.relative_to(DIR_MUSIC)
    output: Path = DIR_MULIPEA.joinpath(relative_input).with_suffix(".m4a")
    output.parent.mkdir(parents=True, exist_ok=True)
    return relative_input, output


# noinspection PyShadowingBuiltins
def _mulipea_convert_any_to_wav(
    input: Path,
    output: Path,
    logfile_io: TextIO,
) -> bool:
    """
    passes the given wav file into ffmpeg, outputs into a temporary file
    before moving it to the final location if command was successful
    """
    _log(
        logfile_io,
        "[mulipea] _mulipea_convert_any_to_wav: running "
        f"`ffmpeg -y -i '{input}' -map 0:a -c:a pcm_s16le -ar 44100 -ac 2 -vn '{output}'`",
    )
    cp: CompletedProcess[bytes] = run(
        [
            "ffmpeg",
            "-y",  # overwrite pls
            # take the input file
            "-i",
            input,
            # take the audio stream
            "-map",
            "0:a",
            # convert to 16-bit pcm (the iPods support <=24 but at that point use alac...)
            "-c:a",
            "pcm_s16le",
            # use 44.1kHz sample rate (the iPods support <=48 but at that point use alac...)
            "-ar",
            "44100",
            # use 2 channels
            "-ac",
            "2",
            # no video pls
            "-vn",
            (output_wav := output.with_suffix(".wav")),
        ],
        stdin=logfile_io,
        stderr=logfile_io,
    )
    if (cp.returncode != 0) or (not output_wav.exists()):
        print(
            f"[mulipea] _mulipea_convert_any_to_wav: error: failed to convert '{input}' to wav",
            file=logfile_io,
            flush=True,
        )
        return False
    _log(logfile_io, "[mulipea] _mulipea_convert_any_to_wav: convert OK")

    # finally, move the file to the final location
    return _helper_move(output_wav, output, logfile_io)


# noinspection PyShadowingBuiltins
def _mulipea_convert_qaac_compatible_to_aac(
    input: Path,
    output: Path,
    logfile_io: TextIO,
) -> bool:
    """
    passes the given file into qaac, outputs into a temporary file
    before moving it to the final location if command was successful
    """
    _log(
        logfile_io,
        "[mulipea] _mulipea_convert_qaac_compatible_to_aac: running "
        f"`qaac --tvbr 127 --quality 2 -o '{output}' '{input}'`",
    )
    cp: CompletedProcess[bytes] = run(
        [
            "qaac",
            "--tvbr",
            "127",
            "--quality",
            "2",
            "-o",
            (output_aac := output.with_suffix(".m4a")),
            input,
        ],
        stdin=logfile_io,
        stderr=logfile_io,
    )
    if (cp.returncode != 0) or (not output_aac.exists()):
        print(
            f"[mulipea] _mulipea_convert_wav_to_aac: error: failed to convert '{input}' to aac",
            file=logfile_io,
            flush=True,
        )
        return False
    _log(logfile_io, "[mulipea] _mulipea_convert_qaac_compatible_to_aac: convert OK")

    # finally, move the file to the final location
    return _helper_move(output_aac, output, logfile_io)


def _mulipea_tag_id3_to_mp4(
    input_metadata: mutagen.FileType,  # pyright: ignore reportPrivateImportUsage
    target_metadata: mutagen.FileType,  # pyright: ignore reportPrivateImportUsage
    logfile_io: TextIO,
) -> None:
    """in-house tag mapper for id3 to mp4 tags, for whatever ffmpeg misses"""
    tags_set_attempted: int = 0
    tags_set_successfully: int = 0

    for input_tag, target_mp4_tag_info in TAGS_MAP_ID3_TO_MP4.items():
        target_mp4_tag, target_mp4_tag_index = target_mp4_tag_info

        # pipeline:
        # go through every tag map we care about
        # - if we have it in the target, then skip it
        # - else, check if we have it in the metadata-full input
        # - apply the tag to the target if we have it in the input
        # - save the target

        if input_tag not in input_metadata:
            continue

        try:
            match input_tag_value := input_metadata[input_tag]:
                # handle these tags specially
                # movement number: NumericPartTextFrame | bpm: NumericTextFrame
                case MVIN() | TBPM():
                    # for the movement number, mp4 doesn't store the total count,
                    # so we just store the index. because of this we can share
                    # logic with the bpm tag
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_id3_to_mp4: (identified MVIN/TBPM) setting tag value - "
                        f"input[{input_tag}]<{type(input_tag_value).__name__}> "
                        f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
                    )
                    tags_set_attempted += 1
                    target_metadata[target_mp4_tag] = +input_tag_value

                # compilation: NumericTextFrame
                case TCMP():
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_id3_to_mp4: (identified TCMP) setting tag value - "
                        f"input[{input_tag}]<{type(input_tag_value).__name__}> "
                        f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
                    )
                    tags_set_attempted += 1
                    target_metadata[target_mp4_tag] = bool(+input_tag_value)

                # track number | disc number: NumericPartTextFrame
                case TRCK() | TPOS():
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_id3_to_mp4: (identified TRCK/TPOS) setting tag pair values - "
                        f"input[{input_tag}]<{type(input_tag_value).__name__}> "
                        f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
                    )
                    tags_set_attempted += 1
                    left, right = input_tag_value.text[0].split("/")
                    target_metadata[target_mp4_tag] = [
                        (
                            int(left),
                            int(right),
                        )
                    ]

                # handle these tags normally
                case TextFrame() | USLT():
                    # check the target if it has the equivalent tag
                    if target_mp4_tag in target_metadata:
                        continue

                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_id3_to_mp4: setting tag value - "
                        f"input[{input_tag}]<{type(input_tag_value).__name__}> "
                        f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
                    )
                    tags_set_attempted += 1

                    # check if the tags a TXXX:TRACKTOTAL or TXXX:DISCTOTAL
                    if input_tag.upper() in ("TXXX:TRACKTOTAL", "TXXX:DISCTOTAL"):
                        target_metadata[target_mp4_tag] = [
                            (
                                target_metadata[target_mp4_tag][0][0],
                                int(input_tag_value.text[0]),
                            )
                        ]

                    elif isinstance(input_tag_value := input_metadata[input_tag], list):
                        target_metadata[target_mp4_tag] = "; ".join(input_tag_value)

                    else:
                        target_metadata[target_mp4_tag] = TAGS_TO_KEEP[target_mp4_tag](
                            input_tag_value
                        )

            tags_set_successfully += 1

        except Exception as e:
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_id3_to_mp4: warning: failed to map tag "
                f"'{input_tag}' to '{target_mp4_tag}' ({e.__class__.__name__}: {e})",
            )
            continue

    try:
        target_metadata.save()
        _log(
            logfile_io,
            "[mulipea] _mulipea_tag_id3_to_mp4: finished, attempted to set "
            f"{tags_set_attempted} tags, {tags_set_successfully} succeeding, "
            f"{tags_set_attempted - tags_set_successfully} failing",
        )
    except Exception as e:
        _log(
            logfile_io,
            "[mulipea] _mulipea_tag_id3_to_mp4: warning: failed to save metadata "
            f"({e.__class__.__name__}: {e})",
        )


def _mulipea_tag_vorbis_to_mp4(
    input_metadata: mutagen.FileType,  # pyright: ignore reportPrivateImportUsage
    target_metadata: mutagen.FileType,  # pyright: ignore reportPrivateImportUsage
    logfile_io: TextIO,
) -> None:
    """in-house tag mapper for vorbis comments to mp4 tags, for whatever ffmpeg misses"""
    tags_set_attempted: int = 0
    tags_set_successfully: int = 0
    part_value: Any

    for input_tag, target_mp4_tag_info in TAGS_MAP_VORBIS_TO_MP4.items():
        target_mp4_tag, target_mp4_tag_index = target_mp4_tag_info

        # pipeline:
        # go through every tag map we care about
        # - if we have it in the target, then skip it
        # - else, check if we have it in the metadata-full input
        # - apply the tag to the target if we have it in the input
        # - save the target

        # the tag is a single value, and we don't have it
        if (target_mp4_tag_index == 0) and (target_mp4_tag not in target_metadata):
            # check if the tag is in the input
            if input_tag not in input_metadata:
                continue

            # if it is, try to map it
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_vorbis_to_mp4: setting tag value - "
                f"input[{input_tag}]<{type(input_metadata[input_tag]).__name__}> "
                f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
            )
            tags_set_attempted += 1

            try:
                if isinstance(input_tag_value := input_metadata[input_tag], list):
                    target_metadata[target_mp4_tag] = "; ".join(input_tag_value)
                else:
                    target_metadata[target_mp4_tag] = TAGS_TO_KEEP[target_mp4_tag](
                        input_tag_value
                    )
                tags_set_successfully += 1
            except Exception as e:
                _log(
                    logfile_io,
                    "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: (map/1) "
                    f"failed to map tag '{input_tag}' to '{target_mp4_tag}' ({e.__class__.__name__}: {e})",
                )
                continue

        # the tag may be a pair, but we just need the first one
        elif target_mp4_tag_index == 1:
            # check if the tag is in the input
            if input_tag not in input_metadata:
                continue

            # if the target already has the tag, skip
            if target_mp4_tag in target_metadata:
                continue

            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_vorbis_to_mp4: setting tag 1st value - "
                f"input[{input_tag}]<{type(input_metadata[input_tag]).__name__}> "
                f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
            )
            tags_set_attempted += 1

            try:
                # figure out what we should set the tag to!
                # if the tag is a collection, grab either what the assumed index of the tag is as
                # per the TAGS_MAP_* dicts, or the last element of whatever collection it is
                if isinstance(
                    input_tag_value := input_metadata[input_tag], (list, tuple)
                ):
                    if len(input_tag_value) != target_mp4_tag_index:
                        _log(
                            logfile_io,
                            "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 1st value"
                            f"of tag {input_tag} to {target_mp4_tag} (expected {target_mp4_tag_index} elements, got {len(input_tag_value)})",
                        )
                        continue
                    part_value = TAGS_TO_KEEP[target_mp4_tag](
                        input_tag_value[
                            min(len(input_tag_value) - 1, target_mp4_tag_index - 1)
                        ]
                    )
                else:
                    part_value = TAGS_TO_KEEP[target_mp4_tag](input_metadata[input_tag])

                # set the tag
                target_metadata[target_mp4_tag] = part_value

                tags_set_successfully += 1

            except Exception as e:
                _log(
                    logfile_io,
                    "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: (map/2) failed to map 1st value"
                    f" of tag '{input_tag}' to '{target_mp4_tag}' ({e.__class__.__name__}: {e})",
                )
                continue

        # the tag is a pair, and we need the second one
        elif target_mp4_tag_index == 2:
            # check if the tag is in the input
            if input_tag not in input_metadata:
                continue

            # if we don't have the first tag, skip
            # (like, would you set max discs without setting the disc number?
            #  imagine a track being number ?/18 like tf)
            target_tag_value = target_metadata.get(target_mp4_tag)
            if not target_tag_value:
                _log(
                    logfile_io,
                    "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 2nd value "
                    f"of tag '{target_mp4_tag}' tag value (missing first value)",
                )
                continue

            # but if we do, we can set the second tag
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_vorbis_to_mp4: setting tag 2nd value - "
                f"input[{input_tag}]<{type(input_metadata[input_tag]).__name__}> "
                f"-> target[{target_mp4_tag}]<{TAGS_TO_KEEP[target_mp4_tag].__name__}>",
            )
            tags_set_attempted += 1

            try:
                # figure out what we should set the tag to!
                # if the tag is a collection, grab either what the assumed index of the tag is as
                # per the TAGS_MAP_* dicts, or the last element of whatever collection it is
                if isinstance(
                    input_tag_value := input_metadata[input_tag], (list, tuple)
                ):
                    # -1 because there has to be a first tag for us to add a second tag.
                    # so if we want to set the 2nd tag, we need at least 1 tag (type shit)
                    if len(input_tag_value) < (target_mp4_tag_index - 1):
                        _log(
                            logfile_io,
                            "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 2nd value "
                            f"of tag {input_tag} to {target_mp4_tag} (expected {target_mp4_tag_index - 1} elements, got {len(input_tag_value)})",
                        )
                        continue

                    part_value = TAGS_TO_KEEP[target_mp4_tag](
                        input_tag_value[
                            min(len(input_tag_value) - 1, target_mp4_tag_index - 1)
                        ]
                    )

                else:
                    part_value = TAGS_TO_KEEP[target_mp4_tag](input_metadata[input_tag])

                # at this point in time, `target_tag_value=[(x[, y],)]`
                # who knows, maybe my files are shit, and it might also be the tuple directly,
                # a la `target_tag_value=(x[, y],)`
                # god knows what it actually is at runtime, so let's check ts out to hell
                if not len(target_tag_value) > 0:
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 2nd value "
                        f"of tag '{target_mp4_tag}' tag value (empty list)",
                    )
                    continue
                # we now guarantee target_tag_value: list[?]

                if not isinstance(target_tag_value[0], tuple):
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 2nd value "
                        f"of tag '{target_mp4_tag}' tag value (collection inside target_tag_value, "
                        f"'{repr(target_tag_value)}' is not a tuple)",
                    )
                    continue
                # we now guarantee target_tag_value: list[tuple]

                if not len(target_tag_value[0]) <= target_mp4_tag_index:
                    _log(
                        logfile_io,
                        "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: skipping mapping 2nd value "
                        f"of tag '{target_mp4_tag}' tag value (collection inside target_tag_value, "
                        f"'{repr(target_tag_value)}' is not a tuple of {target_mp4_tag_index - 1})",
                    )
                    continue
                # we now guarantee target_tag_value: list[tuple[?, ...?]]

                target_metadata[target_mp4_tag] = [
                    (
                        target_tag_value[0][0],
                        part_value,
                    )
                ]
                tags_set_successfully += 1

            except Exception as e:
                _log(
                    logfile_io,
                    "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: (map/3) failed to map 2nd value "
                    f"of tag '{input_tag}' to '{target_mp4_tag}' ({e.__class__.__name__}: {e})",
                )
                continue

    try:
        target_metadata.save()
        _log(
            logfile_io,
            "[mulipea] _mulipea_tag_vorbis_to_mp4: finished, attempted to set "
            f"{tags_set_attempted} tags, {tags_set_successfully} succeeding, "
            f"{tags_set_attempted - tags_set_successfully} failing",
        )

    except Exception as e:
        _log(
            logfile_io,
            "[mulipea] _mulipea_tag_vorbis_to_mp4: warning: failed to save metadata "
            f"({e.__class__.__name__}: {e})",
        )


def _mulipea_tag_mapper(
    input_metadata_full: Path,
    target_m4a: Path,
    logfile_io: TextIO,
) -> mutagen.FileType | None:  # pyright: ignore reportPrivateImportUsage
    """
    transfer as many tags from supported formats to mp4 tags as possible

    currently supports:
    - Vorbis Comments: Ogg Vorbis, FLAC
    - ID3: MP3
    """

    input_metadata: mutagen.FileType  # pyright: ignore reportPrivateImportUsage
    target_metadata: mutagen.FileType  # pyright: ignore reportPrivateImportUsage

    try:
        # load the metadata of the target file
        _target_metadata: mutagen.FileType | None = mutagen.File(target_m4a)  # pyright: ignore reportPrivateImportUsage
        if _target_metadata is None:
            print(
                f"[mulipea] _mulipea_tag_transfer: error: failed to load metadata from '{target_m4a}'",
                file=logfile_io,
                flush=True,
            )
            return None
        target_metadata = _target_metadata
        _log(logfile_io, "[mulipea] _mulipea_tag_transfer: read target metadata OK")

        # load the metadata of the input file
        _input_metadata: mutagen.FileType | None = mutagen.File(input_metadata_full)  # pyright: ignore reportPrivateImportUsage
        if _input_metadata is None:
            print(
                f"[mulipea] _mulipea_tag_transfer: warning: "
                f"target '{input_metadata_full}' had no metadata, skipping...",
                file=logfile_io,
                flush=True,
            )
            return target_metadata
        input_metadata = _input_metadata
        _log(logfile_io, "[mulipea] _mulipea_tag_transfer: read input metadata OK")

    except Exception as e:
        print(
            "[mulipea] _mulipea_tag_transfer: error: "
            f"failed to load metadata from '{input_metadata_full}' ({e.__class__.__name__}: {e})",
            file=logfile_io,
            flush=True,
        )
        return None

    # check the type of the input file and see how to handle it
    match input_metadata:
        case MP3():
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_transfer: matched input metadata via mp3/id3, "
                f"""tags avail are: {",".join([f"{repr(k)}{f' ({len(v)})' if hasattr(v, '__len__') else ''}" for k, v in input_metadata.items()])}""",
            )
            _mulipea_tag_id3_to_mp4(input_metadata, target_metadata, logfile_io)

        # vorbis comment files
        case OggVorbis() | FLAC():
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_transfer: matched input metadata via vorbis comments, "
                f"""tags avail are: {",".join([f"{repr(k)}{f' ({len(v)})' if hasattr(v, '__len__') else ''}" for k, v in input_metadata.items()])}""",
            )
            _mulipea_tag_vorbis_to_mp4(input_metadata, target_metadata, logfile_io)

        case MP4():
            _log(
                logfile_io,
                "[mulipea] _mulipea_tag_transfer: skipping m4a/mp4 file, already has tags (probably)",
            )
            return target_metadata

        case _:
            _log(
                logfile_io,
                f"[mulipea] _mulipea_tag_transfer: warning: skipping '{input_metadata_full}' "
                f"(we dont currently fw '{input_metadata.__class__.__name__}' files)",
            )
            return None

    return target_metadata


def _mulipea_copy_metadata(
    input_metadata: Path,
    intermediate_audio: Path,
    output: Path,
    logfile_io: TextIO,
) -> bool:
    """
    takes metadata from one file, audio from another,
    and outputs a m4a file with the respective metadata and audio
    """

    # 1. run ffmpeg to transfer metadata (that ffmpeg cares about, which isn't much)
    cp: CompletedProcess[bytes]
    if input_metadata.suffix.lower() == ".ogg":
        _log(
            logfile_io,
            "[mulipea] _mulipea_copy_metadata: using new invocation due to file format",
        )
        _log(
            logfile_io,
            "[mulipea] _mulipea_copy_metadata: running "
            f"`ffmpeg -y -i '{input_metadata}' -i '{intermediate_audio}' -codec copy -map 1 -map_metadata 0:s:0 -map 0:1? -disposition:0 default '{output}'`",
        )
        cp = run(
            [
                # this is like my third attempt at a metadata copy command,
                # this one specifically for vorbis commented files
                # (yet the old invocation works on FLACs, so fuck me, I guess)
                "ffmpeg",
                "-y",  # overwrite pls
                # `-i ...` - original metadata-rich file
                "-i",
                input_metadata,
                # `-i ...` - reencoded metadata-less aac file
                "-i",
                intermediate_audio,
                # `-codec copy` - don't touch the audio stream pls
                "-codec",
                "copy",
                # `-map 1` - copy the audio stream from the second input (the metadata-less aac file)
                "-map",
                "1",
                # `-map_metadata 0:s:0` - copy metadata from the first input (the metadata-rich file)
                "-map_metadata",
                "0:s:0",
                # `-map 0:1?` - copy the album art from the first input if it exists
                "-map",
                "0:1?",
                # `-disposition:0 default` set default stream disposition
                "-disposition:0",
                "default",
                output,
            ],
            stdout=logfile_io,
            stderr=logfile_io,
        )

    else:
        _log(
            logfile_io,
            "[mulipea] _mulipea_copy_metadata: using old invocation due to file format",
        )
        _log(
            logfile_io,
            "[mulipea] _mulipea_copy_metadata: running "
            f"`ffmpeg -y -i '{input_metadata}' -i '{intermediate_audio}' -map 1 -codec copy -map 0:1? -map_metadata 0 '{output}'`",
        )
        cp = run(
            [
                "ffmpeg",
                "-y",  # overwrite pls
                # `-i ...` - original metadata-rich file
                "-i",
                input_metadata,
                # `-i ...` - reencoded metadata-less aac file
                "-i",
                intermediate_audio,
                # `-map 1` - copy the audio stream from the second input (the metadata-less aac file)
                "-map",
                "1",
                # `-codec copy` - don't touch the audio stream pls
                "-codec",
                "copy",
                # copy the album art from the first input if it exists
                "-map",
                "0:1?",
                # `-map_metadata 0` - copy metadata from the first input (the metadata-rich file)
                "-map_metadata",
                "0",
                # `-disposition:0 default` - set default stream disposition
                "-disposition:0",
                "default",
                output,
            ],
            stdout=logfile_io,
            stderr=logfile_io,
        )

    if (cp.returncode != 0) or (not output.exists()):
        print(
            "[mulipea] _mulipea_copy_metadata: error: failed to copy metadata "
            f"from '{input_metadata=}' and '{intermediate_audio=}' to '{output=}'",
            file=logfile_io,
            flush=True,
        )
        return False
    _log(logfile_io, "[mulipea] _mulipea_copy_metadata: copy metadata OK")

    # 2. try to match as many tags by ourselves, but we do it after ffmpeg because they know things we don't
    # (this is just here to try to match for like 'extra' tags like sort order, composer, etc.)
    output_metadata = _mulipea_tag_mapper(input_metadata, output, logfile_io)

    # _mulipea_tag_transfer returns the metadata of the output file if it was 'successful',
    # and None if it couldn't even read the metadata of the output file, which we need to check for the bare minimum tags
    if output_metadata is None:
        print(
            f"[mulipea] _mulipea_copy_metadata: error: failed to read metadata from '{output}'",
            file=logfile_io,
            flush=True,
        )
        return False

    return True


def _mulipea_standardise_mp4_metadata(
    original: Path,
    target: Path,
    logfile_io: TextIO,
) -> bool:
    """
    standardise metadata for the target file, either from itself or an original audio file

    only run this when the target can take id3 metadata!

    the given target path is to find for neighbouring cover art and/or a lyric file

    step 0. check for bare minimum tags
    step 1. use mutagen to prune anything unneeded for iPods
    step 2. if there's no cover art, check for a [Cc]over.{jpg|jpeg|png} in the template's directory,
    step 3. if we now have a cover art, resize it to 500x500 or 500 on the larger side
    step 4. `if <template file path>.withsuffix(<.txt or .lrc>)` exists, add them as lyrics
    step 5. swap artist and album artist, including any sort order tags
    """

    _log(
        logfile_io,
        f"[mulipea] _mulipea_standardise_mp4_metadata: original is '{original}'\n"
        f"[mulipea] _mulipea_standardise_mp4_metadata: target is '{target}'",
    )

    possible_failure_reason: str = ""
    try:
        possible_failure_reason = f"failed to load target file '{target}'"
        target_metadata = MP4(target)
        _log(
            logfile_io,
            f"[mulipea] _mulipea_standardise_mp4_metadata: tags avail are: "
            f"""tags avail are: {",".join([f"{repr(k)}{f' ({len(v)})' if hasattr(v, '__len__') else ''}" for k, v in target_metadata.items()])}""",
        )

        # 0. check for bare minimum tags
        bare_minimum: list[bool] = [(tag in target_metadata) for tag in TAGS_MINIMUM]
        if not all(bare_minimum):
            _log(
                logfile_io,
                f"[mulipea] _mulipea_standardise_mp4_metadata: missing bare minimum tags for '{target}'",
            )
            return False

        # step 1. prune
        possible_failure_reason = f"failed to prune tags from target file '{target}'"
        for tag in [tag for tag in target_metadata if (tag not in TAGS_TO_KEEP)]:
            _log(
                logfile_io,
                f"[mulipea] _mulipea_standardise_mp4_metadata: deleting '{tag}'",
            )
            del target_metadata[tag]

        covers: list[MP4Cover] = target_metadata.get("covr", [])  # type: ignore
        _log(logfile_io, f"[mulipea] _mulipea_standardise_mp4_metadata: {len(covers)=}")
        for idx, cover in enumerate(covers):
            if cover.imageformat not in [AtomDataType.JPEG, AtomDataType.PNG]:
                continue

            # resize the cover art with PIL
            # MP4Covers are just bytes with a .imageformat attribute
            possible_failure_reason = f"failed to resize already existing cover art from target file '{target}'"
            cover_image = Image.open(BytesIO(cover))
            cover_image.thumbnail(
                (MAX_LARGEST_COVER_DIMENSION, MAX_LARGEST_COVER_DIMENSION)
            )

            possible_failure_reason = f"failed to save resized, already existing cover art from target file '{target}'"
            cover_image_resized = BytesIO()
            cover_image.save(cover_image_resized, format="JPEG")
            target_metadata["covr"][idx] = MP4Cover(
                cover_image_resized.getvalue(),
                imageformat=AtomDataType.JPEG,
            )

        # step 2. find cover art
        if len(covers) == 0:
            # check for a cover art in the original file's immediate directory
            possible_failure_reason = (
                f"failed to glob for cover art nearby original file '{target}'"
            )
            nearby_images: list[Path] = [
                f
                for f in original.parent.glob("*")
                if (
                    (f.stem.lower() == "cover")
                    and f.suffix.lower() in [".jpg", ".jpeg", ".png"]
                )
            ]
            _log(
                logfile_io,
                f"[mulipea] _mulipea_standardise_mp4_metadata: {len(nearby_images)=}",
            )

            # if there are multiple cover pictures, find the largest one
            possible_failure_reason = (
                f"failed to stat nearby images for original file '{original}'"
            )
            nearby_image_sizes: list[int] = [stat(f).st_size for f in nearby_images]
            nearby_cover: Path | None = (
                nearby_images[nearby_image_sizes.index(max(nearby_image_sizes))]
                if nearby_images
                else None
            )

            if nearby_cover is None:
                print(
                    f"[mulipea] _mulipea_standardise_mp4_metadata: warning: no cover art found nearby '{original}'",
                    file=logfile_io,
                    flush=True,
                )

            else:
                # step 3. standardise cover art: found cover art
                possible_failure_reason = (
                    f"failed to resize found nearby cover art from '{nearby_cover}'"
                )
                cover_image = Image.open(nearby_cover)
                cover_image.thumbnail(
                    (MAX_LARGEST_COVER_DIMENSION, MAX_LARGEST_COVER_DIMENSION)
                )

                possible_failure_reason = f"failed to save resized found nearby cover art from '{nearby_cover}'"
                cover_image_resized = BytesIO()
                cover_image.save(cover_image_resized, format="JPEG")
                target_metadata["covr"] = [
                    MP4Cover(
                        cover_image_resized.getvalue(),
                        imageformat=AtomDataType.JPEG,
                    )
                ]

        # step 4. add lyrics if: there is no lyrics tag, and there is a .txt
        # or .lrc file in the same immediate directory of the original file
        original_unsynced_lyrics: Path = original.with_suffix(".txt")
        original_synced_lyrics: Path = original.with_suffix(".lrc")

        # if it's a synced lyrics file, parse it
        if original_synced_lyrics.exists():
            possible_failure_reason = (
                f"failed to read synced lyrics from '{original_synced_lyrics}'"
            )
            target_metadata["\xa9lyr"] = [
                original_synced_lyrics.read_text(encoding="utf-8")
            ]

        # else, read it wholesale lol
        elif original_unsynced_lyrics.exists():
            possible_failure_reason = (
                f"failed to read lyrics from '{original_unsynced_lyrics}'"
            )
            target_metadata["\xa9lyr"] = [
                original_unsynced_lyrics.read_text(encoding="utf-8")
            ]

        if "\xa9lyr" in target_metadata:
            possible_failure_reason = f"failed to clean up lyrics for '{target}'"
            target_metadata["\xa9lyr"] = _helper_clean_up_lyrics(
                target_metadata["\xa9lyr"][0]
            )

        if SWAP_ARTIST_AND_ALBUM_ARTIST:
            # step 5. swap artist and album artist, and sort order tags if they exist
            # "\xa9ART": str,  # artist
            # "aART": str,  # album artist
            # "soaa": str,  # album artist sort order
            # "soar": str,  # artist sort order
            possible_failure_reason = (
                f"failed to swap artist and album artist for '{target}'"
            )
            # _log(
            #     logfile_io,
            #     "[mulipea] _mulipea_standardise_mp4_metadata:\n"
            #     f"... {target_metadata.get("\xa9aRT", "undefined")=}"
            #     f"... {target_metadata.get("aART", "undefined")=}"
            #     f"... {target_metadata.get("soaa", "undefined")=}"
            #     f"... {target_metadata.get("soar", "undefined")=}",
            # )
            # return False

            if ("\xa9ART" in target_metadata) and ("aART" in target_metadata):
                target_metadata["\xa9ART"], target_metadata["aART"] = (
                    target_metadata["aART"],
                    target_metadata["\xa9ART"],
                )
            if ("soaa" in target_metadata) and ("soar" in target_metadata):
                target_metadata["soaa"], target_metadata["soar"] = (
                    target_metadata["soar"],
                    target_metadata["soaa"],
                )

        if "\xa9too" in target_metadata:
            encoder = target_metadata["\xa9too"]
            target_metadata["\xa9too"] = (
                [f"Mulipea Converter {__version__} via {encoder[0]}"]
                if isinstance(encoder, list)
                else f"Mulipea Converter {__version__} via {encoder}"
            )

        # finally, save the metadata
        possible_failure_reason = "failed to save audio"
        target_metadata.save()

    except Exception as e:
        print(
            "[mulipea] _mulipea_standardise_mp4_metadata: error: "
            f"{possible_failure_reason} ({e.__class__.__name__}: {e})",
            file=logfile_io,
            flush=True,
        )
        return False

    return True


# noinspection PyShadowingBuiltins
def _mulipea_handle_aac(
    input: Path,
    rinput: Path,
    output: Path,
    logfile_io: TextIO,
    progress_dict: "DictProxy[int, ProgressUpdate] | None" = None,
    progress_task_id: int | None = None,
) -> bool:
    """if the file's already an aac, just copy it over"""

    with TemporaryDirectory() as tmpdir:
        intermediate = Path(tmpdir).joinpath(rinput)
        intermediate.parent.mkdir(parents=True, exist_ok=True)
        _log(
            logfile_io,
            f"[mulipea] _mulipea_handle_aac: intermediate is '{intermediate}'",
        )

        _log(logfile_io, "[mulipea] _mulipea_handle_aac: copy to intermediate")
        if not _helper_copy(
            input,
            intermediate,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_aac: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_aac: standardising metadata")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="standardising metadata",
            ),
            progress_dict,
            progress_task_id,
            completed=(1 / 1),
        )
        if not _mulipea_standardise_mp4_metadata(
            input,
            intermediate,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_aac: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_aac: moving to output")
        if not _helper_move(
            intermediate,
            output,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_aac: failed")
            return False

    _log(logfile_io, "[mulipea] _mulipea_handle_aac: all ok")
    return True


# noinspection PyShadowingBuiltins
def _mulipea_handle_alac(
    input: Path,
    rinput: Path,
    output: Path,
    logfile_io: TextIO,
    progress_dict: "DictProxy[int, ProgressUpdate] | None" = None,
    progress_task_id: int | None = None,
) -> bool:
    """if it's alac, qaac willl happily convert it to a high-quality m4a"""

    with TemporaryDirectory() as tmpdir:
        intermediate_final = Path(tmpdir).joinpath(rinput).with_suffix(".m4a")
        intermediate_final.parent.mkdir(parents=True, exist_ok=True)
        intermediate_metadataless = intermediate_final.with_suffix(".metadataless.m4a")

        _log(
            logfile_io,
            f"[mulipea] _mulipea_handle_alac: intermediate_metadataless is '{intermediate_metadataless}'\n"
            f"[mulipea] _mulipea_handle_alac: intermediate_final is '{intermediate_final}'",
        )

        _log(logfile_io, "[mulipea] _mulipea_handle_alac: converting to aac")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="converting to aac",
            ),
            progress_dict,
            progress_task_id,
            completed=(1 / 3),
        )
        if not _mulipea_convert_qaac_compatible_to_aac(
            input,
            intermediate_metadataless,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_alac: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_alac: copying metadata")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="copying metadata",
            ),
            progress_dict,
            progress_task_id,
            completed=(2 / 3),
        )
        if not _mulipea_copy_metadata(
            input,
            intermediate_metadataless,
            intermediate_final,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_alac: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_alac: standardising metadata")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="standardising metadata",
            ),
            progress_dict,
            progress_task_id,
            completed=(3 / 1),
        )
        if not _mulipea_standardise_mp4_metadata(
            input,
            intermediate_final,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_alac: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_alac: moving to output")
        if not _helper_move(
            intermediate_final,
            output,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_alac: failed")
            return False

    _log(logfile_io, "[mulipea] _mulipea_handle_alac: all ok")
    return True


# noinspection PyShadowingBuiltins
def _mulipea_handle_generic(
    input: Path,
    rinput: Path,
    output: Path,
    logfile_io: TextIO,
    progress_dict: "DictProxy[int, ProgressUpdate] | None" = None,
    progress_task_id: int | None = None,
) -> bool:
    """if it isn't aac or alac, convert it to wav, convert that to aac, then copy and standardise metadata"""

    with TemporaryDirectory() as tmpdir:
        intermediate_wav = Path(tmpdir).joinpath(rinput).with_suffix(".wav")
        intermediate_wav.parent.mkdir(parents=True, exist_ok=True)
        intermediate_metadataless_aac = intermediate_wav.with_suffix(
            ".metadataless.m4a"
        )
        intermediate_final = intermediate_wav.with_suffix(".m4a")

        _log(
            logfile_io,
            f"[mulipea] _mulipea_handle_generic: intermediate_wav is '{intermediate_wav}'\n"
            f"[mulipea]: _mulipea_handle_generic: intermediate_metadataless_aac is '{intermediate_metadataless_aac}'\n"
            f"[mulipea] _mulipea_handle_generic: intermediate_final is '{intermediate_final}'",
        )

        _log(logfile_io, "[mulipea] _mulipea_handle_generic: converting to wav")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="converting to wav",
            ),
            progress_dict,
            progress_task_id,
            completed=(1 / 4),
        )
        if not _mulipea_convert_any_to_wav(
            input,
            intermediate_wav,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_generic: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_generic: converting to aac")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="converting to aac",
            ),
            progress_dict,
            progress_task_id,
            completed=(1 / 4),
        )
        if not _mulipea_convert_qaac_compatible_to_aac(
            intermediate_wav,
            intermediate_metadataless_aac,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_generic: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_generic: copying metadata")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}", right="copying metadata"
            ),
            progress_dict,
            progress_task_id,
            completed=(3 / 4),
        )
        if not _mulipea_copy_metadata(
            input,
            intermediate_metadataless_aac,
            intermediate_final,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_generic: failed")
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_generic: standardising metadata")
        _helper_update_progress(
            _helper_make_progress_task_text(
                left=f" - {input.name}",
                right="standardising metadata",
            ),
            progress_dict,
            progress_task_id,
            completed=(4 / 4),
        )
        if not _mulipea_standardise_mp4_metadata(
            input,
            intermediate_final,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_generic: failed")
            try:
                if output.exists():
                    remove(output)
            except Exception as e:
                _log(
                    logfile_io,
                    "[mulipea] _mulipea_handle_generic: failed to clean up output "
                    f"'{output}' ({e.__class__.__name__}: {e})",
                )
            return False

        _log(logfile_io, "[mulipea] _mulipea_handle_generic: moving to output")
        if not _helper_move(
            intermediate_final,
            output,
            logfile_io,
        ):
            _log(logfile_io, "[mulipea] _mulipea_handle_generic: failed")
            return False

    return True


# noinspection PyShadowingBuiltins
def mulipea_encode(
    input: Path,
    logfile: Path,
    progress_dict: "DictProxy[int, ProgressUpdate] | None" = None,
    progress_task_id: int | None = None,
) -> None:
    logfile_io = logfile.open("a+", encoding="utf-8")

    codec = _mulipea_get_m4a_codec(input, logfile_io)
    _log(
        logfile,
        f"[mulipea] mulipea_encode: {codec=} ({input.suffix=})",
    )
    handling_function: Callable[
        [Path, Path, Path, TextIO, DictProxy[int, ProgressUpdate], int],
        bool,
    ]
    if codec == "aac":
        handling_function = _mulipea_handle_aac
    elif codec == "alac":
        handling_function = _mulipea_handle_alac
    else:
        handling_function = _mulipea_handle_generic

    rinput, output = _mulipea_common_prehandle(input)
    attempts: int = 1

    if output.exists():
        _dprint(f"debug: mulipea_encode: '{output}' already exists, skipping")
        try:
            logfile_io.close()
            remove(logfile)
        except Exception as e:
            _dprint(
                f"debug: mulipea_encode: failed to remove '{logfile}' ({e.__class__.__name__}: {e})"
            )
        if (progress_dict is not None) and (progress_task_id is not None):
            progress_dict[progress_task_id] = ProgressUpdate(remove=True)
        return

    while attempts <= MAX_RETRY_ATTEMPTS:
        if attempts > 1:
            _log(
                logfile,
                f"[mulipea: retrying, below is attempt #{attempts}]",
            )

        if handling_function(
            input, rinput, output, logfile_io, progress_dict, progress_task_id
        ):
            # if we're here, everything went well, remove the log file
            _dprint(f"debug: mulipea_encode: success, removing '{logfile}'")
            try:
                logfile_io.close()
                remove(logfile)
            except Exception as e:
                _dprint(
                    f"debug: mulipea_encode: failed to remove '{logfile}' ({e.__class__.__name__}: {e})"
                )
            break

        else:
            attempts += 1

        logfile_io.write("\n")

    else:
        _log(logfile, "[mulipea: failed]")
        logfile_io.close()

    if (progress_dict is not None) and (progress_task_id is not None):
        progress_dict[progress_task_id] = ProgressUpdate(remove=True)


def main() -> None:
    from hashlib import md5
    from multiprocessing import Process
    from os import cpu_count
    from time import sleep

    from rich import print as rprint  # type: ignore
    from rich.progress import TaskID  # type: ignore

    rprint(
        f"[bold]Mulipea Converter {__version__}[/bold]\n"
        f" - target directory: '{DIR_MUSIC}'\n"
        f" - output directory: '{DIR_MULIPEA}'\n"
    )

    targets: list[Path] = []
    encountered_files: int = 0
    for file in DIR_MUSIC.rglob("*"):
        if not (file.is_file() and file.suffix in SUPPORTED_FILE_TYPES):
            continue
        if (
            DIR_MULIPEA.joinpath(file.relative_to(DIR_MUSIC))
            .with_suffix(".m4a")
            .exists()
        ):
            continue
        targets.append(file)
        encountered_files += 1

    if len(targets) == 0:
        rprint(
            "[bold green]yay!!![/bold green] nothing to do... "
            f"all {encountered_files} files have already been converted... zzz..."
        )
        exit(0)

    _cores = cpu_count()
    # workers = max(1, _cores if (_cores is not None) else 1)
    workers = max(
        1, int((_cores if (_cores is not None) else 1) * PROCESSOR_SATURATION)
    )
    processes: list[Process] = []

    with Manager() as manager:
        with Progress() as progress:
            overall_task = progress.add_task(
                f"processing {len(targets)} files",
                total=len(targets),
            )
            process_updates: DictProxy[int, ProgressUpdate] = manager.dict()

            # process every file
            for i, file in enumerate(targets, start=1):
                # make each logfile path
                process_logfile: Path = DIR_LOGS.joinpath(
                    f"{md5(str(file.relative_to(DIR_MUSIC)).encode('utf-8')).hexdigest()[:8]} {file.stem}.log"
                )
                if not process_logfile.parent.exists():
                    process_logfile.parent.mkdir(parents=True, exist_ok=True)

                # wait for enough processes to die if we're at our limit of processes
                while len(processes) >= workers:
                    # stop keeping track of dead processes
                    for process in processes:
                        if not process.is_alive():
                            processes.remove(process)
                            progress.update(
                                overall_task,
                                completed=i,
                                description=f"processed {i}/{len(targets)} files",
                            )
                            break

                    # update the progress bars
                    for process_task_id, progress_update in process_updates.items():
                        task_id = TaskID(process_task_id)

                        if progress_update.remove:
                            progress.remove_task(task_id)
                            del process_updates[task_id]
                        else:
                            progress.update(
                                task_id,
                                total=progress_update.total,
                                completed=progress_update.completed,
                                advance=progress_update.advance,
                                description=progress_update.description,
                                visible=progress_update.visible,
                                refresh=progress_update.refresh,
                            )

                    sleep(0.1)

                # now that there's room for processes, process another file
                progress_task_id = progress.add_task(
                    _helper_make_progress_task_text(
                        left=f" - {file.name}", right="getting codec"
                    ),
                    total=None,
                    # total=1.0,
                )
                process = Process(
                    target=mulipea_encode,
                    args=(file, process_logfile, process_updates, progress_task_id),
                )
                process.start()
                processes.append(process)

            # wait for finished processes
            while processes:
                for process in processes:
                    if not process.is_alive():
                        processes.remove(process)
                        break

                for process_task_id, progress_update in process_updates.items():
                    task_id = TaskID(process_task_id)
                    if progress_update.remove:
                        progress.remove_task(task_id)
                        del process_updates[task_id]
                    else:
                        progress.update(
                            task_id,
                            total=progress_update.total,
                            completed=progress_update.completed,
                            advance=progress_update.advance,
                            description=progress_update.description,
                            visible=progress_update.visible,
                            refresh=progress_update.refresh,
                        )

                progress.update(
                    overall_task,
                    total=None,
                    description=f"finishing up... ({len(processes)} processes remain)",
                )

                sleep(0.5)

    # check log dir if any .log file exists, the number of equates to how many failed
    failed: list[Path] = [f for f in DIR_LOGS.rglob("*.log")]
    if failed:
        rprint(
            f"\n[bold red]nay...[/bold red] failed to encode {len(failed)} files, check '{DIR_LOGS}' for more info"
        )
        for file in targets:
            would_be_logfile_name: str = f"{md5(str(file.relative_to(DIR_MUSIC)).encode('utf-8')).hexdigest()[:8]} {file.stem}.log"
            if DIR_LOGS.joinpath(would_be_logfile_name).exists():
                rprint(f" - {file.relative_to(DIR_MUSIC)}")
    else:
        rprint("\n[bold green]yay!!![/bold green] all files processed a-okay")


if __name__ == "__main__":
    main()
