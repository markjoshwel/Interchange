"""
daycircle: parser for daycircle plaintext files
-----------------------------------------------
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

from enum import Enum
from sys import stderr
from typing import NamedTuple

from .utils import Result

# daycircle file format pseudo-grammar
# ------------------------------------
# root     = metadata
#          | key <whitespace> value
#          ;
# metadata = "day" <whitespace> date
#          | "colour" <whitespace> rgbhex
#          ;
# key      = <string>
#          | "@" <string>
#          ;
# value    = time           # single time for @<key> (marker) events
#          | time "-" time  # time range for <key> (range) events
#          ;
# date     = <0-9> <0-9> "-"          # dd-
#            <0-9> <0-9> "-"          # mm-
#            <0-9> <0-9> <0-9> <0-9>  # yyyy
#          ;
# time     = <0-9> <0-9> <0-9> <0-9>  # 24h time format (e.g, 0000, 2359)
#          ;
# rgbhex   = <0-F> <0-F> <0-F> <0-F> <0-F> <0-F>  # 6-digit hex colour code
#          ;


class DaycircleKey(Enum):
    DAY = "day"
    COLOUR = "#"
    EVENT_MARKER = "@"
    EVENT_RANGE = "*"
    UNKNOWN = ""


class DaycircleDate(NamedTuple):
    day: int
    month: int
    year: int

    @staticmethod
    def from_str(date: str) -> Result["DaycircleDate"]:
        # dd-mm-yyyy
        split_date = date.split("-")

        if len(split_date) == 3:
            day, month, year = split_date

            if day.isdigit() and month.isdigit() and year.isdigit():
                return Result[DaycircleDate](
                    DaycircleDate(int(day), int(month), int(year))
                )

        return Result[DaycircleDate](
            DaycircleDate(0, 0, 0),
            error=ValueError(f"invalid date format: {date}"),
        )

    def __str__(self) -> str:
        return f"{self.day:02}-{self.month:02}-{self.year:04}"


class DaycircleColour(NamedTuple):
    code: str

    @staticmethod
    def from_str(code: str) -> Result["DaycircleColour"]:
        if len(code) == 6 and all([char in "0123456789ABCDEFabcdef" for char in code]):
            return Result[DaycircleColour](DaycircleColour(code))
        else:
            return Result[DaycircleColour](
                DaycircleColour(""), error=ValueError(f"invalid colour code: {code}")
            )

    def __str__(self) -> str:
        return "#" + self.code[:6]


class DaycircleTime(NamedTuple):
    hour: int
    minute: int

    @staticmethod
    def from_str(time: str) -> Result["DaycircleTime"]:
        if time.isdigit() and (len(time) == 4):
            return Result[DaycircleTime](DaycircleTime(int(time[:2]), int(time[2:])))
        else:
            return Result[DaycircleTime](
                DaycircleTime(0, 0), error=ValueError(f"invalid time format: {time}")
            )

    def __str__(self) -> str:
        return f"{self.hour:02}{self.minute:02}"


class DaycircleEventMarker(NamedTuple):
    name: str
    time: DaycircleTime

    def __hash__(self) -> int:
        return hash(self.name)


class DaycircleEventRange(NamedTuple):
    name: str
    start: DaycircleTime
    end: DaycircleTime

    def __hash__(self) -> int:
        return hash(self.name)


DaycircleEvent = DaycircleEventMarker | DaycircleEventRange


class DaycircleFileData(NamedTuple):
    day: DaycircleDate | None = None
    event_colours: dict[str, DaycircleColour] = {}
    events: list[DaycircleEvent] = []


def parse(
    content: str, filename: str = "", is_colour_file: bool = False
) -> Result[DaycircleFileData]:
    day: DaycircleDate | None = None
    event_colours: dict[str, DaycircleColour] = {}
    events: list[DaycircleEvent] = []

    for line in content.splitlines():
        key: str = ""
        key_type: DaycircleKey = DaycircleKey.UNKNOWN
        value: str = ""

        # key handling
        match line.strip().split(maxsplit=1):
            case [DaycircleKey.DAY.value, value]:
                key_type = DaycircleKey.DAY

            case [key, value]:
                if key.startswith(DaycircleKey.EVENT_MARKER.value):
                    key = key.lstrip(DaycircleKey.EVENT_MARKER.value)
                    key_type = DaycircleKey.EVENT_MARKER

                elif key.startswith(DaycircleKey.COLOUR.value):
                    key = key.lstrip(DaycircleKey.COLOUR.value)
                    key_type = DaycircleKey.COLOUR

                else:
                    key_type = DaycircleKey.EVENT_RANGE

        # value handling
        match key_type:
            case DaycircleKey.DAY:  # dd-mm-yyyy string
                if date_result := DaycircleDate.from_str(value):
                    day = date_result.get()

            case DaycircleKey.COLOUR:  # rgb hex code
                if colour_result := DaycircleColour.from_str(value):
                    event_colours[key] = colour_result.get()

            case DaycircleKey.EVENT_MARKER:  # time
                if time_result := DaycircleTime.from_str(value):
                    events.append(DaycircleEventMarker(key, time_result.get()))

            case DaycircleKey.EVENT_RANGE:  # time range (time-time)
                if len(time_range := value.split("-")) == 2:
                    start_result = DaycircleTime.from_str(time_range[0])
                    end_result = DaycircleTime.from_str(time_range[1])

                    if start_result and end_result:
                        events.append(
                            DaycircleEventRange(
                                key,
                                start_result.get(),
                                end_result.get(),
                            )
                        )

    if day is None:
        if is_colour_file:
            return Result[DaycircleFileData](
                DaycircleFileData(
                    day=DaycircleDate(0, 0, 0), event_colours=event_colours, events=events
                ),
            )

        else:
            return Result[DaycircleFileData](
                DaycircleFileData(
                    day=DaycircleDate(0, 0, 0),
                    event_colours=event_colours,
                    events=events,
                ),
                error=ValueError(
                    "missing day metadata"
                    + (f" for file '{filename}'" if filename else "")
                ),
            )

    else:
        return Result[DaycircleFileData](
            DaycircleFileData(day=day, event_colours=event_colours, events=events)
        )
