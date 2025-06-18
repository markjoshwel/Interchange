# sota staircase SideStepper
# a somewhat fast .gitignore-respecting large file finder
# licence: 0BSD

from dataclasses import dataclass
from functools import cache
from multiprocessing import Manager, cpu_count

# noinspection PyProtectedMember
from multiprocessing.managers import ListProxy
from os import getenv
from os.path import abspath
from pathlib import Path
from subprocess import CompletedProcess
from subprocess import run as _run
from sys import argv, executable, stderr
from textwrap import indent
from time import time
from traceback import format_tb
from typing import Final, Generator, Generic, Iterable, Iterator, NamedTuple, TypeVar

# constants
INDENT = "   "
REPO_DIR: Final[Path] = Path(__file__).parent
REPO_SOTAIGNORE: Final[Path] = REPO_DIR.joinpath(".sotaignore")
_SOTA_SIDESTEP_CHUNK_SIZE = getenv("SIDESTEP_CHUNK_SIZE")
SOTA_SIDESTEP_CHUNK_SIZE: Final[int] = (
    int(_SOTA_SIDESTEP_CHUNK_SIZE)
    if (
        (_SOTA_SIDESTEP_CHUNK_SIZE is not None)
        and (_SOTA_SIDESTEP_CHUNK_SIZE.isdigit())
    )
    else 16
)
_SOTA_SIDESTEP_MAX_WORKERS = getenv("SIDESTEP_MAX_WORKERS")
SOTA_SIDESTEP_MAX_WORKERS: Final[int] = (
    int(_SOTA_SIDESTEP_MAX_WORKERS)
    if (
        (_SOTA_SIDESTEP_MAX_WORKERS is not None)
        and (_SOTA_SIDESTEP_MAX_WORKERS.isdigit())
    )
    else cpu_count()
)
SOTA_SIDESTEP_LARGE_FILE_SIZE: Final[int] = 100000000  # 100mb
SOTA_SIDESTEP_PARALLEL: Final[bool] = getenv("SIDESTEP_PARALLEL") is not None


# define these before importing third-party modules because we use them in the import check
def generate_command_failure_message(cp: CompletedProcess) -> str:
    return "\n".join(
        [
            f"\n\nfailure: command '{cp.args}' failed with exit code {cp.returncode}",
            f"{INDENT}stdout:",
            (
                indent(text=cp.stdout.decode(), prefix=f"{INDENT}{INDENT}")
                if (isinstance(cp.stdout, bytes) and (cp.stdout != b""))
                else f"{INDENT}{INDENT}(no output)"
            ),
            f"{INDENT}stderr:",
            (
                indent(text=cp.stderr.decode(), prefix=f"{INDENT}{INDENT}")
                if (isinstance(cp.stderr, bytes) and (cp.stderr != b""))
                else f"{INDENT}{INDENT}(no output)"
            )
            + "\n",
        ]
    )


def run(
    command: str | list,
    cwd: Path | str | None = None,
    capture_output: bool = True,
    give_input: str | None = None,
) -> CompletedProcess:
    """
    exception-safe-ish wrapper around subprocess.run()

    args:
        command: str | list
            the command to run
        cwd: Path | str | None = None
            the working directory
        capture_output: bool = True
            whether to capture the output

    returns: CompletedProcess
        the return object from subprocess.run()
    """

    # noinspection PyBroadException
    try:
        cp = _run(
            command,
            shell=False if isinstance(command, list) else True,
            cwd=cwd,
            capture_output=capture_output,
            input=give_input.encode() if give_input else None,
        )
    except Exception as run_exc:
        print(
            f"\n\nfailure: command '{command}' failed with exception",
            f"{INDENT}{run_exc.__class__.__name__}: {run_exc}",
            indent(text="\n".join(format_tb(run_exc.__traceback__)), prefix=INDENT),
            sep="\n",
        )
        exit(-1)
    return cp


# attempt to import third-party modules
# if they're not installed, prompt the user to optionally install them automatically
_could_not_import: list[str] = []
_could_not_import_exc: Exception | None = None

try:
    from gitignore_parser import IgnoreRule, rule_from_pattern  # type: ignore
except ImportError as _import_exc:
    _could_not_import.append("gitignore_parser")
    _could_not_import_exc = _import_exc

try:
    # noinspection PyUnresolvedReferences
    from tqdm import tqdm

    # noinspection PyUnresolvedReferences
    from tqdm.contrib.concurrent import process_map
except ImportError as _import_exc:
    _could_not_import.append("tqdm")
    _could_not_import_exc = _import_exc

if _could_not_import:
    for module in _could_not_import:
        print(
            f"critical error: '{module}' is not installed, "
            f"please run 'pip install {module}' to install it",
        )

    # install the missing modules
    if input("\ninstall these with pip? y/n: ").lower() == "y":
        print("installing...", end="", flush=True)
        _cp = run([executable, "-m", "pip", "install", *_could_not_import])
        if _cp.returncode != 0:
            print(generate_command_failure_message(_cp))
            exit(-1)
        print(" done", flush=True)

    # check if they were installed successfully
    _cp = run(
        [
            executable,
            "-c",
            ";".join([f"import {module}" for module in _could_not_import]),
        ]
    )
    if _cp.returncode != 0:
        print(generate_command_failure_message(_cp))

        print(
            "critical error: post-install check failed. reverting installation...",
            end="",
            flush=True,
        )
        _cp = run([executable, "-m", "pip", "uninstall", *_could_not_import, "-y"])
        if _cp.returncode != 0:
            print(generate_command_failure_message(_cp))
        print(" done", flush=True)

        exit(-1)

    elif __name__ == "__main__":
        # rerun the script if we're running as one
        exit(
            run(
                [executable, Path(__file__).absolute(), *argv[1:]], capture_output=False
            ).returncode
        )

    else:
        # we're being imported, raise an error
        raise EnvironmentError(
            "automatic dependency installation successful"
        ) from _could_not_import_exc

A = TypeVar("A")
B = TypeVar("B")


class OneSided(Generic[A, B], NamedTuple):
    """
    generic tuple with two elements, a and b, given by a generator
    in which element 'a' is a constant and b is from an iterable/iterator
    """

    a: A
    b: B


def one_sided(a: A, bbb: Iterable[B]) -> Iterator[OneSided[A, B]]:
    """
    generator that yields OneSided instances with a constant 'a' element
    and elements from the given iterable/iterator 'bbb' as the 'b' element
    """
    for b in bbb:
        yield OneSided(a, b)


def generate_time_elapsed_string(time_taken: float) -> str:
    """generates a human-readable time-elapsed string from a time taken float"""
    hours = int(time_taken // 3600)
    minutes = int(time_taken % 3600 // 60)
    seconds = int(time_taken % 60)

    time_taken_string: str

    if time_taken > 3600:
        time_taken_string = f"{hours}h {minutes}′ {seconds}″"
    elif time_taken > 60:
        time_taken_string = f"{minutes}′ {seconds}″"
    else:
        time_taken_string = f"{time_taken:.2f}″"

    return time_taken_string


@dataclass(eq=True, frozen=True)
class SideStepIgnoreMatcher:
    """immutable gitignore matcher"""

    root: Path
    # (
    #     (.gitignore file directory path, (ignore rule, ...)),
    #     (.gitignore file directory path, (ignore rule, ...)),
    #     ...
    # )
    rules: tuple[tuple[Path, tuple[IgnoreRule, ...]], ...] = tuple()

    def add_gitignore(self, gitignore: Path) -> "SideStepIgnoreMatcher":
        """returns a new SidestepIgnoreMatcher with rules from the given gitignore file"""

        new_ruleset: list[IgnoreRule] = []
        for line_no, line_text in enumerate(gitignore.read_text().splitlines()):
            rule = rule_from_pattern(
                pattern=line_text.rstrip("\n"),
                base_path=Path(abspath(gitignore.parent)),
                source=(gitignore, line_no),
            )
            if rule:
                new_ruleset.append(rule)

        return SideStepIgnoreMatcher(
            root=self.root, rules=self.rules + ((gitignore.parent, tuple(new_ruleset)),)
        )

    def match(self, file: Path | str) -> bool:
        """returns True if the file is ignored by any of the rules in the gitignore files, False otherwise"""
        matched = False

        # check to see if the gitignore affects the file
        for ignore_dir, ruleset in self.rules:
            if str(ignore_dir) not in str(file):
                continue
            if not self._possibly_negated(ruleset):
                matched = matched or any(r.match(file) for r in ruleset)
            else:
                for rule in reversed(ruleset):
                    if rule.match(file):
                        matched = matched or not rule.negation
        return matched

    def match_trytrytry(self, file: Path) -> Path | None:
        """
        same as match, but also checks if the gitignore files ignore any parent directories;
        horribly slow and dumb, thus the name 'trytrytry'

        returns the ignored parent path if the file is ignored, None otherwise
        """

        trytrytry: Path = file
        while trytrytry != trytrytry.parent:
            if self.match(trytrytry):
                return trytrytry
            if len(self.root.parts) == len(trytrytry.parts):
                return None
            trytrytry = trytrytry.parent
        return None

    @cache
    def _possibly_negated(self, ruleset: tuple[IgnoreRule, ...]) -> bool:
        return any(rule.negation for rule in ruleset)


@dataclass(eq=True, frozen=True)
class LargeFileFilterResult:
    """
    result data structure of the large file filter

    files: tuple[Path, ...]
        large files found
    matcher: SideStepIgnoreMatcher
        the *ignore matcher instance
    ignore_directories: tuple[Path, ...]
        directories that were ignored
    """

    files: tuple[Path, ...]
    matcher: SideStepIgnoreMatcher
    ignore_directories: tuple[Path, ...]


def _parallel() -> bool:
    """
    helper function to determine if we should use multiprocessing;
    checks the environment variable SIDESTEP_PARALLEL and the command line arguments

    returns: bool
    """
    if SOTA_SIDESTEP_PARALLEL:
        return True
    elif "--parallel" in argv:
        return True
    return False


def _iter_files(
    target: Path,
    pattern: str = "*",
) -> Generator[Path, None, None]:
    """
    generator that yields files in the target directory excluding '.git/**'

    args:
        target: Path
            the directory to search in
        pattern: str = "*"
            the file pattern to search for

    yields: Path
        file in the target directory
    """
    repo_dir = target.joinpath(".git/")
    for target_file in target.rglob(pattern):
        if not target_file.is_file():
            continue
        if repo_dir in target_file.parents:
            continue
        yield target_file


def iter_files(target_dir: Path) -> tuple[tuple[Path, ...], SideStepIgnoreMatcher]:
    """
    get all non-git files and register .gitignore files

    args:
        target_dir: Path
            the directory to search in

    returns: tuple[tuple[Path, ...], SideStepIgnoreMatcher]
        tuple of all files in the target directory and a SideStepIgnoreMatcher instance
    """

    all_files: list[Path] = []
    sim = SideStepIgnoreMatcher(root=target_dir)

    for file in tqdm(
        _iter_files(target_dir),
        desc="1 pre | finding large files - scanning (1/3)",
        leave=False,
    ):
        all_files.append(file)
        if file.name == ".gitignore":
            sim = sim.add_gitignore(file)

    return tuple(all_files), sim


def _filter_sim_match(
    os: OneSided[tuple[list[Path], SideStepIgnoreMatcher], Path],
) -> Path | None:
    """first filter pass function, thread-safe-ish"""
    (ignore_dirs, sim), file = os.a, os.b

    ignored = False
    for ign_dir in ignore_dirs:
        if str(ign_dir) in str(file):
            ignored = True
            break

    if (not ignored) and ((ttt := sim.match_trytrytry(file)) is not None):
        if ttt.is_dir() and ttt not in ignore_dirs:
            ignore_dirs.append(ttt)
        return None
    return file


def _filter_ign_dirs_and_size(os: OneSided[list[Path], Path]) -> Path | None:
    """second filter pass function, thread-safe-ish"""
    ignore_dirs, file = os.a, os.b

    for ign_dir in ignore_dirs:
        if str(ign_dir) in str(file):
            return None
    else:
        # we're here because the file is not ignored by any of the rules
        # (the 'else' clause is only executed if the for loop completes without breaking)
        if file.stat().st_size > SOTA_SIDESTEP_LARGE_FILE_SIZE:
            return file
        return None


def _find_large_files_single(
    files: tuple[Path, ...], sim: SideStepIgnoreMatcher
) -> LargeFileFilterResult:
    """single-process implementation of find_large_files"""
    ignore_dirs: list[Path] = []

    _files = []
    for fsm_os in tqdm(
        one_sided(a=(ignore_dirs, sim), bbb=files),
        desc="1 pre | finding large files - iod-ttt file matching (2/3)",
        leave=False,
        total=len(files),
    ):
        if f := _filter_sim_match(fsm_os):
            _files.append(f)

    large_files = []
    for fds_os in tqdm(
        one_sided(a=ignore_dirs, bbb=_files),
        desc="1 pre | finding large files - dir rematching (3/3)",
        leave=False,
        total=len(_files),
    ):
        f = _filter_ign_dirs_and_size(fds_os)
        if f is not None:
            large_files.append(f)

    return LargeFileFilterResult(
        files=tuple(large_files),
        matcher=sim,
        ignore_directories=tuple(ignore_dirs),
    )


def _find_large_files_parallel(
    files: tuple[Path, ...], sim: SideStepIgnoreMatcher
) -> LargeFileFilterResult:
    """multiprocess implementation of find_large_files"""
    manager = Manager()
    ignore_dirs: ListProxy[Path] = manager.list()

    _files: list[Path] = [
        f
        for f in process_map(
            _filter_sim_match,
            one_sided(a=(ignore_dirs, sim), bbb=files),
            desc="1 pre | finding large files - iod-ttt file matching (2/3)",
            leave=False,
            chunksize=SOTA_SIDESTEP_CHUNK_SIZE,
            max_workers=SOTA_SIDESTEP_MAX_WORKERS,
            total=len(files),
        )
        if f is not None
    ]

    large_files: tuple[Path, ...] = tuple(
        [
            f
            for f in process_map(
                _filter_ign_dirs_and_size,
                one_sided(a=ignore_dirs, bbb=_files),
                desc="1 pre | finding large files - dir rematching (3/3)",
                leave=False,
                chunksize=SOTA_SIDESTEP_CHUNK_SIZE,
                max_workers=SOTA_SIDESTEP_MAX_WORKERS,
                total=len(files),
            )
            if f is not None
        ]
    )

    return LargeFileFilterResult(
        files=large_files,
        matcher=sim,
        ignore_directories=tuple(ignore_dirs),
    )


def find_large_files(
    files: tuple[Path, ...], matcher: SideStepIgnoreMatcher
) -> LargeFileFilterResult:
    """
    finds all files larger than a certain size in a directory;
    uses SOTA_SIDESTEP_LARGE_FILE_SIZE as the size threshold

    args:
        files: tuple[Path, ...]
            list of files to search through
        matcher: SideStepIgnoreMatcher
            the ignore matcher instance from iter_files()

    returns: LargeFileFilterResult
    """
    if _parallel():
        return _find_large_files_parallel(files, matcher)
    else:
        return _find_large_files_single(files, matcher)


def write_sotaignore(large_files: tuple[Path, ...]) -> bool:
    """
    writes out a .sotaignore file with a list of large files,
    updating an existing one if already present

    args:
        large_files: list[Path]
            list of large files

    returns: bool
        True if anything was written, False otherwise (no changes)
    """
    if not large_files:
        return False

    old_sotaignore = (
        REPO_SOTAIGNORE.read_text().strip().splitlines()
        if REPO_SOTAIGNORE.exists()
        else []
    )

    new_sotaignore = [ln for ln in old_sotaignore] + [
        lf.relative_to(REPO_DIR).as_posix()
        for lf in large_files
        if lf.relative_to(REPO_DIR).as_posix() not in old_sotaignore
    ]

    if new_sotaignore == old_sotaignore:
        return False

    # check if the sotaignore file starts with a comment
    if new_sotaignore and not new_sotaignore[0].startswith("#"):
        for line in [
            "# .sotaignore file generated by sota staircase ReStepper/SideStepper",
            "# anything here either can't or shouldn't be uploaded github",
            "# unless you know what you're doing, don't edit this file! >:(",
        ][::-1]:
            new_sotaignore.insert(0, line)

    REPO_SOTAIGNORE.touch(exist_ok=True)
    REPO_SOTAIGNORE.write_text("\n".join(new_sotaignore) + "\n")
    return True


def main() -> None:
    """command-line entry function"""

    print(
        "\nsota staircase SideStepper",
        f"   repo root   : {REPO_DIR.relative_to(Path.cwd())}",
        (
            f"   .sotaignore : {REPO_SOTAIGNORE.relative_to(Path.cwd())} "
            f"({'exists' if REPO_SOTAIGNORE.exists() else 'does not exist'})"
        ),
        f"   parallel?   : {'yes' if _parallel() else 'no'}\n",
        sep="\n",
        file=stderr,
    )

    cumulative_start_time = time()

    print(f"1/3{INDENT}pre-scanning repository... ", end="", file=stderr)
    start_time = time()
    files, sim = iter_files(REPO_DIR)
    end_time = time()
    print(
        f"1/3{INDENT}pre-scanning repository... "
        f"done in {generate_time_elapsed_string(end_time - start_time)} "
        f"(found {len(files)})",
        file=stderr,
    )

    print(f"2/3{INDENT}finding large files... ", end="", file=stderr)
    start_time = time()
    large_files = find_large_files(files, sim).files
    end_time = time()
    print(
        f"2/3{INDENT}finding large files... "
        f"done in {generate_time_elapsed_string(end_time - start_time)} "
        f"(found {len(large_files)})",
        file=stderr,
    )

    print(f"3/3{INDENT}writing .sotaignore file... ", end="", file=stderr)
    start_time = time()
    was_written = write_sotaignore(large_files)
    end_time = time()
    print(
        ("done" if was_written else "skipped")
        + f" in {generate_time_elapsed_string(end_time - start_time)}\n",
        file=stderr,
    )

    for file in large_files:
        print(file.relative_to(REPO_DIR).as_posix())

    cumulative_end_time = time()
    print(
        f"\n--- done! took {generate_time_elapsed_string(cumulative_end_time - cumulative_start_time)}~ "
        "☆*: .｡. o(≧▽≦)o .｡.:*☆ ---",
        flush=True,
        file=stderr,
    )


if __name__ == "__main__":
    main()
