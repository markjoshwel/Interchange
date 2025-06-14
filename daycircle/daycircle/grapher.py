"""
daycircle.grapher: graphing logic for daycircle
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

from io import BytesIO
from pathlib import Path
from typing import NamedTuple

import matplotlib.font_manager as font_manager  # type: ignore
import matplotlib.pyplot as plt  # type: ignore
import seaborn as sns  # type: ignore
from matplotlib.font_manager import FontProperties, fontManager  # type: ignore
from matplotlib.patches import Arc, Patch, Rectangle  # type: ignore
from matplotlib.transforms import Affine2D  # type: ignore

from .parser import (
    DaycircleColour,
    DaycircleDate,
    DaycircleEvent,
    DaycircleEventMarker,
    DaycircleEventRange,
    DaycircleTime,
)
from .utils import result

colour_palette: list[str] = [
    # using singaporean sun cycle
    *sns.color_palette("blend:#0e0c09,#080605", 5).as_hex(),  # 0000-0400
    *["#4f454b", "#f6b697", "#d5bd9e"],  # 0500, 0600, 0700
    *sns.color_palette("blend:#b2bbaf,#7c96a5", 5).as_hex(),  # 0800-1200
    *["#7c96a5"] * 5,  # 1300-1700
    *sns.color_palette("blend:#272f42,#080605", 3).as_hex(),  # 1800-2000
    *sns.color_palette("blend:#080605,#0e0c09", 3).as_hex(),  # 2100-2300
]


class DaycircleGraphData(NamedTuple):
    """namedtuple representing a daycircle graph"""

    date: DaycircleDate | None = None
    date_end: DaycircleDate | None = None
    event_colours: dict[str, DaycircleColour] = {}
    events: list[DaycircleEvent] = []

    def to_filename(
        self, name_override: Path | str | None = None, file_type: str = "svg"
    ) -> str:
        """return a filename for this graph"""
        working_dir: Path = Path()
        filename: str = f"graph.{file_type}"
        override: bool = False

        if isinstance(name_override, Path):
            if name_override.is_dir():
                working_dir = name_override
            else:
                if name_override.parent.is_dir():
                    working_dir = name_override.parent
                filename = f"{name_override.name}.{file_type}"
                override = True

        elif name_override is not None:
            filename = f"{name_override}.{file_type}"

        if (not override) and (self.date is not None):
            filename = "{start}{end}.{type}".format(
                start=self.date,
                end=self.date_end if (self.date_end is not None) else "",
                type=file_type,
            )

        return str(working_dir.joinpath(filename))


def time_to_deg(time: DaycircleTime) -> float:
    """convert an hour to a degree, where 1200h is 0deg, 1800h is 90deg, etc."""
    h, m = time.hour % 24, time.minute % 60
    # counterclockwise starting from 1800
    dh = (270 - (h * 15)) + (360 if h >= 18 else 0)
    dm = -(15 * (m / 60))
    return dh + dm


@result(default=BytesIO())
def graph(
    data: DaycircleGraphData,
    event_colours: dict[str, DaycircleColour] = {},
    font_path: Path | None = None,
    format: str = "svg",
) -> BytesIO:
    """graph a daycircle"""

    if isinstance(font_path, Path):
        fontManager.addfont(font_path)
        font = FontProperties(fname=font_path)
        plt.rcParams["font.family"] = font.get_name()

    # plot chart
    plt.figure(num=f"daycircle: {data.to_filename(file_type='')[:-1]}", figsize=(8, 8))
    sns.set_palette("husl")
    wedges, *_ = (
        plt.pie(
            [1] * 24,
            labels=[f"{h}" for h in range(24)],
            startangle=-(6 * 15) + (15 / 2),
            counterclock=False,
            colors=colour_palette,
            radius=2.5,
            wedgeprops={"linewidth": None, "linestyle": ""},
        ),
    )
    plt.axis("equal")

    def add_range_arc(
        event: DaycircleEventRange,
        colour: str,
    ) -> None:
        pos_x: float = 0
        pos_y: float = 0
        arrow_radius: float = 3
        arrow_angle: float = 0
        arrow_start_angle = time_to_deg(event.start)
        arrow_end_angle = time_to_deg(event.end)

        arrow = Arc(
            xy=(pos_x, pos_y),
            width=arrow_radius,
            height=arrow_radius,
            angle=0,
            theta1=arrow_end_angle,
            theta2=arrow_start_angle,
            color=str(colour),
            lw=40,
            capstyle="butt",
            zorder=10,
        )

        axs = plt.gca()
        axs.add_patch(arrow)

    def add_marker_line(
        event: DaycircleEventMarker,
        colour: str,
    ) -> None:
        line_height = 0.025
        line_width = 2.5
        line = Rectangle(
            xy=(0, 0 - (line_height * 0.5)),
            width=line_width,
            height=line_height,
            zorder=20,
            color=str(colour),
            angle=time_to_deg(event.time),
            rotation_point=(0, 0),
            capstyle="round",
        )
        axs = plt.gca()
        axs.add_patch(line)

    events_range = [e for e in data.events if isinstance(e, DaycircleEventRange)]
    events_marker = [e for e in data.events if isinstance(e, DaycircleEventMarker)]
    events_legend: dict[str, Patch] = {}

    for revent, fallback_colour in zip(
        events_range,
        sns.color_palette("pastel6", len(events_range)).as_hex(),
    ):
        add_range_arc(
            revent,
            colour=str(event_colours.get(revent.name, fallback_colour)),
        )
        events_legend[revent.name] = Patch(
            label=revent.name,
            color=str(event_colours.get(revent.name, fallback_colour)),
            capstyle="round",
        )

    for mevent, fallback_colour in zip(
        events_marker,
        sns.color_palette("husl", len(events_marker)).as_hex(),
    ):
        add_marker_line(
            mevent,
            colour=str(event_colours.get(mevent.name, fallback_colour)),
        )
        events_legend[mevent.name] = Patch(
            label=mevent.name,
            color=str(event_colours.get(mevent.name, fallback_colour)),
            capstyle="round",
        )

    plt.legend(
        handles=events_legend.values(),
        loc="lower center",
        bbox_to_anchor=(0, -0.05),
        frameon=False,
    )

    # save chart
    buffer = BytesIO()
    plt.savefig(buffer, format=format)

    # show chart
    # plt.show()

    return buffer
