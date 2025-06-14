from enum import Enum

import win32com.client as win32
from sys import stderr
from csv import reader
from pathlib import Path
from typing import NamedTuple, Any, Generator

SIZE_LEN_TENDENCY_ARROW: float = 515.0
SIZE_LEN_DISTRIBUTION_ARROW: float = 600.0
SIZE_VIS_CIRCLE: float = 170.0

EXPORT_PREFIX: str = "afterlife-"
EXPORT_SUFFIX: str = ""
TARGET_LAYER: str = "Working"

DIR_OUTPUT: Path = Path(__file__).parent.joinpath("output")


class AfterlifeValues(NamedTuple):
    # seven deadly sins
    lust: float
    gluttony: float
    greed: float
    sloth: float
    envy: float
    wrath: float
    pride: float

    # seven heavenly virtues
    chastity: float
    temperance: float
    charity: float
    diligence: float
    kindness: float
    patience: float
    humility: float

    # the number of responses used to calculate this
    n: int


class AfterlifeInformation(NamedTuple):
    name: str
    results: AfterlifeValues
    results_male_only: AfterlifeValues
    results_female_only: AfterlifeValues
    results_other_only: AfterlifeValues


class InformationOriginType(Enum):
    CUMULATIVE = "all"
    MALE = "male pure"
    FEMALE = "female pure"
    OTHER = "other pure"


class AiTransformation(Enum):
    # https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=7d83f8592174c956d45892b11e310e5db5e45353, page 267
    aiTransformBottom = 7
    aiTransformBottomLeft = 4
    aiTransformBottomRight = 10
    aiTransformCenter = 6
    aiTransformDocumentOrigin = 1
    aiTransformLeft = 3
    aiTransformRight = 9
    aiTransformTop = 5
    aiTransformTopLeft = 2
    aiTransformTopRight = 8


class AiZOrderMethod(Enum):
    # https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=7d83f8592174c956d45892b11e310e5db5e45353, page 268
    aiBringForward = 2
    aiBringToFront = 1
    aiSendBackward = 3
    aiSendToBack = 4


def parse_csv(path: Path) -> Generator[AfterlifeInformation, None, None]:
    # read from 'detailed.csv'
    # ... ['name', 'name', 'gender', 'sins', '', '', '', '', '', '', 'virtues', '', '', '', '', '', '', 'n']
    # ... ['', '', '', 'lust', 'gluttony', 'greed', 'sloth', 'envy', 'wrath', 'pride', 'chastity', 'temperance', 'charity', 'diligence', 'kindness', 'patience', 'humility', '']
    # ... ...
    # ... ['mark', 'mark', 'all', '3.40', '2.56', '2.51', '3.49', '2.51', '1.98', '3.02', '4.69', '3.51', '4.64', '3.80', '4.47', '4.62', '3.71', '16']
    # ... ['mark', '', 'male pure', '3.69', '2.28', '2.44', '3.59', '2.47', '1.78', '2.78', '4.59', '3.19', '4.25', '3.34', '4.38', '4.53', '3.34', '16']
    # ... ['mark', '', 'male adj', '2.62', '1.62', '1.73', '2.56', '1.76', '1.27', '1.98', '3.27', '2.27', '3.02', '2.38', '3.11', '3.22', '2.38', '16']
    # ... ['mark', '', 'female pure', '2.82', '3.09', '2.64', '3.64', '2.91', '2.36', '3.73', '4.91', '4.18', '5.55', '4.73', '4.45', '4.64', '4.55', '4']
    # ... ['mark', '', 'female adj', '0.69', '0.76', '0.64', '0.89', '0.71', '0.58', '0.91', '1.20', '1.02', '1.36', '1.16', '1.09', '1.13', '1.11', '4']
    # ... ['mark', '', 'other pure', '2.00', '4.00', '3.00', '1.00', '1.00', '3.00', '3.00', '5.00', '5.00', '6.00', '6.00', '6.00', '6.00', '5.00', '1']
    # ... ['mark', '', 'other adj', '0.09', '0.18', '0.13', '0.04', '0.04', '0.13', '0.13', '0.22', '0.22', '0.27', '0.27', '0.27', '0.27', '0.22', '1']
    # ...

    with open(path, "r") as file:
        data = reader(file)

        # ignore the first two rows and 'adj' rows
        next(data)
        next(data)

        person: int = 0
        name: str | None = None
        results: AfterlifeValues | None = None
        results_male_only: AfterlifeValues | None = None
        results_female_only: AfterlifeValues | None = None
        results_other_only: AfterlifeValues | None = None

        for idx, row in enumerate(data):
            # calculate what person every 7 rows is a new person
            _person = int(idx / 7)
            if _person != person:
                assert name is not None
                assert results is not None
                assert results_male_only is not None
                assert results_female_only is not None
                assert results_other_only is not None
                yield AfterlifeInformation(
                    name=name,
                    results=results,
                    results_other_only=results_other_only,
                    results_female_only=results_female_only,
                    results_male_only=results_male_only,
                )
                person = _person

            # parse line
            name = row[0]
            origin: InformationOriginType

            if not name:
                continue
            elif (_origin := row[2]) in [e.value for e in InformationOriginType]:
                origin = InformationOriginType(_origin)
            else:
                continue

            _results = AfterlifeValues(
                lust=float(row[3]),
                gluttony=float(row[4]),
                greed=float(row[5]),
                sloth=float(row[6]),
                envy=float(row[7]),
                wrath=float(row[8]),
                pride=float(row[9]),
                chastity=float(row[10]),
                temperance=float(row[11]),
                charity=float(row[12]),
                diligence=float(row[13]),
                kindness=float(row[14]),
                patience=float(row[15]),
                humility=float(row[16]),
                n=int(row[17]),
            )

            match origin:
                case InformationOriginType.CUMULATIVE:
                    results = _results
                case InformationOriginType.MALE:
                    results_male_only = _results
                case InformationOriginType.FEMALE:
                    results_female_only = _results
                case InformationOriginType.OTHER:
                    results_other_only = _results
                case _:
                    assert False, "supposedly unreachable code"

        assert name is not None
        assert results is not None
        assert results_male_only is not None
        assert results_female_only is not None
        assert results_other_only is not None
        yield AfterlifeInformation(
            name=name,
            results=results,
            results_other_only=results_other_only,
            results_female_only=results_female_only,
            results_male_only=results_male_only,
        )


def printingpress(data: AfterlifeInformation, document: Any) -> None:
    # get groups
    working_layer = document.Layers(TARGET_LAYER)
    header_layer = working_layer.GroupItems("Header")
    numbers_layer = working_layer.GroupItems("Numbers")
    vis_lust_chastity_layer = working_layer.GroupItems("LustChastity")
    vis_gluttony_temperance_layer = working_layer.GroupItems("GluttonyTemperance")
    vis_greed_charity_layer = working_layer.GroupItems("GreedCharity")
    vis_sloth_diligence_layer = working_layer.GroupItems("SlothDiligence")
    vis_wrath_patience_layer = working_layer.GroupItems("WrathPatience")
    vis_envy_kindness_layer = working_layer.GroupItems("EnvyKindness")
    vis_pride_humility_layer = working_layer.GroupItems("PrideHumility")

    print(
        f"afterlife.printingpress({data.name}): operating on the document...",
        file=stderr,
        flush=True,
    )

    # set header and numbers
    # - set 'TargetName' text box in 'Working' > 'Header'
    # - set 'All' text box in 'Working' > 'Numbers'
    # - set 'Male' text box in 'Working' > 'Numbers'
    # - set 'Female' text box in 'Working' > 'Numbers'
    # - set 'Other' text box in 'Working' > 'Numbers'
    header_layer.TextFrames("TargetName").Contents = data.name
    numbers_layer.TextFrames("All").Contents = str(data.results.n)
    numbers_layer.TextFrames("Male").Contents = str(data.results_male_only.n)
    numbers_layer.TextFrames("Female").Contents = str(data.results_female_only.n)
    numbers_layer.TextFrames("Other").Contents = str(data.results_other_only.n)

    def circle_size(
        value_score: float,
    ) -> tuple[float, float, float, float, float, float]:
        # calculate the circle sizes for each value,
        # - e.g., for a score of 2.8, 'Left1' and 'Left2' remain their maximum size, 'Left3' is 80% of the maximum size, and so on
        c6: float = max(value_score - 5, 0)
        c5: float = max(value_score - c6 - 4, 0)
        c4: float = max(value_score - (c6 + c5) - 3, 0)
        c3: float = max(value_score - (c6 + c5 + c4) - 2, 0)
        c2: float = max(value_score - (c6 + c5 + c4 + c3) - 1, 0)
        c1: float = max(value_score - (c6 + c5 + c4 + c3 + c2), 0)
        return c1, c2, c3, c4, c5, c6

    def transform(
        item: Any,
        to_width: int | float,
        to_height: int | float,
        origin: AiTransformation,
    ) -> None:
        # from adobe illustrator scripting reference:
        # ... Transform
        # ...     (transformationMatrix as Matrix,
        # ...     [, changePositions as Boolean]
        # ...     [, changeFillPatterns as Boolean]
        # ...     [, changeFillGradients as Boolean]
        # ...     [, changeStrokePattern as Boolean]
        # ...     [, changeLineWidths as Double]
        # ...     [, transformAbout as AiTransformation])

        # NOTE: btw a few things break if you scale objects to 0
        # for some things like lines, it works, but especially on PathItems
        # like circles, it borks tf out of it and has to be manually fixed...
        # so usually if you're scaling to 0, just set the opacity to 0

        # calculate scaling sx, sy
        sx = (to_width / item.Width) if item.Width != 0 else to_width
        sy = (to_height / item.Height) if item.Height != 0 else to_height

        change_positions: bool = True
        change_fill_patterns: bool = False
        change_fill_gradients: bool = False
        change_stroke_pattern: bool = False
        change_line_widths: float = 0.0

        matrix = win32.Dispatch("Illustrator.Matrix")
        matrix.MValueA = sx
        matrix.MValueB = 0.0
        matrix.MValueC = 0.0
        matrix.MValueD = sy
        matrix.MValueTX = 0.0
        matrix.MValueTY = 0.0

        item.Transform(
            matrix,
            change_positions,
            change_fill_patterns,
            change_fill_gradients,
            change_stroke_pattern,
            change_line_widths,
            origin.value,
        )

    def z_order(item: Any, z: int) -> None:
        item.ZOrder(AiZOrderMethod.aiBringToFront.value)
        for _ in range(z):
            item.ZOrder(AiZOrderMethod.aiSendBackward.value)

    for pair_layer, left, right in [
        [vis_lust_chastity_layer, data.results.lust, data.results.chastity],
        [vis_gluttony_temperance_layer, data.results.gluttony, data.results.temperance],
        [vis_greed_charity_layer, data.results.greed, data.results.charity],
        [vis_sloth_diligence_layer, data.results.sloth, data.results.diligence],
        [vis_wrath_patience_layer, data.results.wrath, data.results.patience],
        [vis_envy_kindness_layer, data.results.envy, data.results.kindness],
        [vis_pride_humility_layer, data.results.pride, data.results.humility],
    ]:
        print(
            f"afterlife.printingpress({data.name}): setting scores for {pair_layer.Name}...",
            file=stderr,
            flush=True,
        )

        # set left and right scores for each sin/virtue pair
        # - e.g. set 'Lust' text box in 'Working' > 'LustChastity' > 'LeftScore'
        # - e.g. set 'Chastity' text box in 'Working' > 'LustChastity' > 'RightScore'
        pair_layer.TextFrames("LeftScore").Contents = f"{left:.2f}"
        pair_layer.TextFrames("RightScore").Contents = f"{right:.2f}"

        # set sum scores for each sin/virtue pair
        # - e.g. set 'SumScore' text box in 'Working' > 'LustChastity' > 'SumScore'
        sum_score = right - left
        pair_layer.TextFrames("SumScore").Contents = f"{sum_score:.2f}"

        print(
            f"afterlife.printingpress({data.name}): setting circle sizes for {pair_layer.Name}...",
            file=stderr,
            flush=True,
        )

        for side_data, side_name in zip([left, right], ["Left", "Right"]):
            for idx, c_size in zip(range(1, 7), circle_size(side_data)):
                circle = pair_layer.PathItems(f"{side_name}{idx}")
                if c_size == 0:
                    circle.Opacity = 0.0
                else:
                    circle.Opacity = 100.0
                    transform(
                        circle,
                        SIZE_VIS_CIRCLE * c_size,
                        SIZE_VIS_CIRCLE * c_size,
                        AiTransformation.aiTransformCenter,
                    )

        # set tendency arrows
        # - e.g. set 'LustChastity' > 'LeftTendency'
        # - e.g. set 'LustChastity' > 'RightTendency'
        #
        # if the sum score is -1, LeftTendency is set to (abs(-1)/6) * SIZE_LEN_TENDENCY_ARROW and RightTendency hidden
        # if the sum score is 2.8, LeftTendency hidden and RightTendency is set to (2.8/6) * SIZE_LEN_TENDENCY_ARROW

        print(
            f"afterlife.printingpress({data.name}): setting tendency arrows...",
            file=stderr,
            flush=True,
        )

        if sum_score > 0:
            pair_layer.PathItems("LeftTendency").Opacity = 0
            pair_layer.PathItems("RightTendency").Opacity = 100
            transform(
                pair_layer.PathItems("RightTendency"),
                (sum_score / 6) * SIZE_LEN_TENDENCY_ARROW,
                0,
                AiTransformation.aiTransformLeft,
            )

        elif sum_score == 0:
            pair_layer.PathItems("LeftTendency").Opacity = 100
            pair_layer.PathItems("RightTendency").Opacity = 100
            transform(
                pair_layer.PathItems("LeftTendency"),
                0.01,
                0,
                AiTransformation.aiTransformRight,
            )
            transform(
                pair_layer.PathItems("RightTendency"),
                0.01,
                0,
                AiTransformation.aiTransformLeft,
            )

        else:
            pair_layer.PathItems("LeftTendency").Opacity = 100
            pair_layer.PathItems("RightTendency").Opacity = 0
            transform(
                pair_layer.PathItems("LeftTendency"),
                (abs(sum_score) / 6) * SIZE_LEN_TENDENCY_ARROW,
                0,
                AiTransformation.aiTransformRight,
            )

    # set response gender makeup arrows
    for layer, _left, _right in [
        [
            vis_lust_chastity_layer,
            [
                # data.results.lust,
                data.results_male_only.lust,
                data.results_female_only.lust,
                data.results_other_only.lust,
            ],
            [
                # data.results.chastity,
                data.results_male_only.chastity,
                data.results_female_only.chastity,
                data.results_other_only.chastity,
            ],
        ],
        [
            vis_gluttony_temperance_layer,
            [
                # data.results.gluttony,
                data.results_male_only.gluttony,
                data.results_female_only.gluttony,
                data.results_other_only.gluttony,
            ],
            [
                # data.results.temperance,
                data.results_male_only.temperance,
                data.results_female_only.temperance,
                data.results_other_only.temperance,
            ],
        ],
        [
            vis_greed_charity_layer,
            [
                # data.results.greed,
                data.results_male_only.greed,
                data.results_female_only.greed,
                data.results_other_only.greed,
            ],
            [
                # data.results.charity,
                data.results_male_only.charity,
                data.results_female_only.charity,
                data.results_other_only.charity,
            ],
        ],
        [
            vis_sloth_diligence_layer,
            [
                # data.results.sloth,
                data.results_male_only.sloth,
                data.results_female_only.sloth,
                data.results_other_only.sloth,
            ],
            [
                # data.results.diligence,
                data.results_male_only.diligence,
                data.results_female_only.diligence,
                data.results_other_only.diligence,
            ],
        ],
        [
            vis_wrath_patience_layer,
            [
                # data.results.wrath,
                data.results_male_only.wrath,
                data.results_female_only.wrath,
                data.results_other_only.wrath,
            ],
            [
                # data.results.patience,
                data.results_male_only.patience,
                data.results_female_only.patience,
                data.results_other_only.patience,
            ],
        ],
        [
            vis_envy_kindness_layer,
            [
                # data.results.envy,
                data.results_male_only.envy,
                data.results_female_only.envy,
                data.results_other_only.envy,
            ],
            [
                # data.results.kindness,
                data.results_male_only.kindness,
                data.results_female_only.kindness,
                data.results_other_only.kindness,
            ],
        ],
        [
            vis_pride_humility_layer,
            [
                # data.results.pride,
                data.results_male_only.pride,
                data.results_female_only.pride,
                data.results_other_only.pride,
            ],
            [
                # data.results.humility,
                data.results_male_only.humility,
                data.results_female_only.humility,
                data.results_other_only.humility,
            ],
        ],
    ]:
        print(
            f"afterlife.printingpress({data.name}): setting makeup arrows for {layer.Name}...",
            file=stderr,
            flush=True,
        )

        # set makeup arrows for each sin/virtue pair (contd.)
        # ... - e.g., set 'LustChastity' > 'LeftMakeup' > 'Male' | 'Female' | 'Other' to width of max 100% * SIZE_LEN_DISTRIBUTION_ARROW
        # ... if:
        # ...   male responses avg at 3.82
        # ...   female responses avg at 4.02
        # ...   other responses avg at 2.00,
        # ... then
        # ...   'Male' arrow is (3.82/6) * SIZE_LEN_DISTRIBUTION_ARROW
        # ...   'Female' arrow is (4.02/6) * SIZE_LEN_DISTRIBUTION_ARROW
        # ...   'Other' arrow is (2.00/6) * SIZE_LEN_DISTRIBUTION_ARROW
        # ... and
        # ...   pre-step: all are z-ordered to the bottom
        # ...   female (longest) is kept at the bottom
        # ...   male (second longest) is moved up
        # ...   other (shortest) is moved up twice, basically on top
        # ... (on each side: left and right for their respective sin/virtue value pair)

        left_makeup_dict: dict[str, float] = dict(
            sorted(
                {
                    "Male": _left[0],
                    "Female": _left[1],
                    "Other": _left[2],
                }.items(),
                key=lambda item: item[1],
            )
        )

        right_makeup_dict: dict[str, float] = dict(
            sorted(
                {
                    "Male": _right[0],
                    "Female": _right[1],
                    "Other": _right[2],
                }.items(),
                key=lambda item: item[1],
            )
        )

        respondents = {
            "Male": data.results_male_only.n,
            "Female": data.results_female_only.n,
            "Other": data.results_other_only.n,
        }

        for side, makeup in zip(
            ["Left", "Right"], [left_makeup_dict, right_makeup_dict]
        ):
            side_group = layer.GroupItems(f"{side}Makeup")

            for idx, (gender, score) in enumerate(makeup.items(), start=1):
                arrow = side_group.PathItems(gender)

                if respondents.get(gender, 0) <= 0.0:
                    arrow.Opacity = 0.0
                else:
                    arrow.Opacity = 100.0
                    transform(
                        arrow,
                        (score / 6) * SIZE_LEN_DISTRIBUTION_ARROW,
                        0,
                        AiTransformation.aiTransformRight
                        if side == "Left"
                        else AiTransformation.aiTransformLeft,
                    )
                    z_order(arrow, idx)

    # the second final step: export
    def export(name: str, additional: str = "") -> None:
        filename: str = f"{EXPORT_PREFIX}{name}{EXPORT_SUFFIX}{additional}.png"

        print(
            f"afterlife.printingpress({data.name}): exporting '{filename}'...",
            file=stderr,
            flush=True,
        )

        # define export options
        options = win32.Dispatch("Illustrator.ExportOptionsPNG24")
        options.AntiAliasing = True
        options.ArtBoardClipping = True
        options.Transparency = False

        DIR_OUTPUT.mkdir(exist_ok=True)

        document.Export(
            DIR_OUTPUT.joinpath(filename),
            5,  # png
            options,
        )

    export(f"{data.name}")

    # the actual final step: remove all text and arrows, and re-export
    # ... hide layer 'Working' > 'Numbers'
    # ... hide layer ['LustChastity' ...] > 'SumScore' (text)
    # ... hide layer ['LustChastity' ...] > 'Left' (text)
    # ... hide layer ['LustChastity' ...] > 'LeftScore' (text)
    # ... hide layer ['LustChastity' ...] > 'LeftMakeup' (group)
    # ... hide layer ['LustChastity' ...] > 'LeftTendency' (arrow)
    # ... hide layer ['LustChastity' ...] > 'Right' (text)
    # ... hide layer ['LustChastity' ...] > 'RightScore' (text)
    # ... hide layer ['LustChastity' ...] > 'RightMakeup' (group)
    # ... hide layer ['LustChastity' ...] > 'RightTendency' (arrow)

    def hide_non_shapes(value: bool) -> None:
        numbers_layer.Hidden = value

        for _layer in [
            vis_lust_chastity_layer,
            vis_gluttony_temperance_layer,
            vis_greed_charity_layer,
            vis_sloth_diligence_layer,
            vis_wrath_patience_layer,
            vis_envy_kindness_layer,
            vis_pride_humility_layer,
        ]:
            _layer.TextFrames("SumScore").Hidden = value
            _layer.TextFrames("Left").Hidden = value
            _layer.TextFrames("LeftScore").Hidden = value
            _layer.GroupItems("LeftMakeup").Hidden = value
            _layer.PathItems("LeftTendency").Hidden = value
            _layer.TextFrames("Right").Hidden = value
            _layer.TextFrames("RightScore").Hidden = value
            _layer.GroupItems("RightMakeup").Hidden = value
            _layer.PathItems("RightTendency").Hidden = value

    print(
        f"afterlife.printingpress({data.name}): making variant 2...",
        file=stderr,
        flush=True,
    )
    hide_non_shapes(True)
    # the 'blend' object group
    # don't know why i can't access it by name
    # like if you add a breakpoint, the .Name attribute is '' (empty string)
    # weird...
    working_layer.PluginItems(1).Hidden = False
    export(f"{data.name}", additional="-var2")

    print(
        f"afterlife.printingpress({data.name}): making variant 1...",
        file=stderr,
        flush=True,
    )
    hide_non_shapes(True)
    working_layer.PluginItems(1).Hidden = True
    export(f"{data.name}", additional="-var1")

    print(
        f"afterlife.printingpress({data.name}): reverting...",
        file=stderr,
        flush=True,
    )
    hide_non_shapes(False)
    working_layer.PluginItems(1).Hidden = False

    print(f"afterlife.printingpress({data.name}): done", file=stderr)


def main() -> None:
    print(
        "afterlife: hooking into illustrator...",
        file=stderr,
        flush=True,
    )

    ai = win32.GetActiveObject("Illustrator.Application")
    assert ai, "could not hook into adobe illustrator"

    print(
        "afterlife: leave any of the following blank for their defaults",
        file=stderr,
    )

    csvpath = "detailed.csv"
    while (Path(csvpath).exists() and Path(csvpath).is_file()) is False:
        csvpath = input("   path to csv file (default: 'detailed.csv'): ")

    global EXPORT_PREFIX, EXPORT_SUFFIX, TARGET_LAYER
    _prefix = input(f"   export prefix (default: '{EXPORT_PREFIX}'): ")
    _suffix = input(f"   export suffix (default: '{EXPORT_SUFFIX}'): ")
    _target = input(f"   target layer  (default: '{TARGET_LAYER}'): ")

    EXPORT_PREFIX = _prefix if _prefix != "" else EXPORT_PREFIX
    EXPORT_SUFFIX = _suffix if _suffix != "" else EXPORT_SUFFIX
    TARGET_LAYER = _target if _target != "" else TARGET_LAYER

    data: list[AfterlifeInformation] = [i for i in parse_csv(Path(csvpath))]
    print(f"afterlife: loaded {len(data)} entries", file=stderr)

    names: list[str] = sorted(i.name.lower() for i in data)
    print(
        "\ndata available for:\n",
        "\n".join(f"   {name}" for name in names),
        "\n",
        sep="",
        file=stderr,
    )

    query = ""
    while (query not in names) and (query != "*"):
        query = input("> ").lower()

    if query == "*":
        for p in data:
            printingpress(p, document=ai.ActiveDocument)
    else:
        printingpress(
            [i for i in data if i.name.lower() == query][0], document=ai.ActiveDocument
        )

    print("afterlife: done", file=stderr)


if __name__ == "__main__":
    main()
