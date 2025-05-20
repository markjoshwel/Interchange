# sota staircase ReStepper
# forge -> github one-way repo sync script
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

from json import loads as json_loads

# noinspection PyProtectedMember
from multiprocessing.pool import ThreadPool
from os import X_OK, access, cpu_count, getenv
from pathlib import Path
from platform import machine, system
from pprint import pformat
from shutil import copy2
from subprocess import CompletedProcess
from subprocess import run as _run
from sys import argv, executable, stderr
from tempfile import TemporaryDirectory
from textwrap import indent
from time import time
from traceback import format_tb
from typing import Callable, Final, TypeVar
from urllib.request import urlopen

# constants
INDENT: Final[str] = "   "
VERBOSE: Final[bool] = "--verbose" in argv
REPO_URL_GITHUB: Final[str] = "github.com/markjoshwel/wirm"
REPO_URL_FORGE: Final[str] = "forge.joshwel.co/mark/wirm"
COMMIT_MESSAGE: Final[str] = "chore(restep): sync with forge"
COMMIT_AUTHOR: Final[str] = "sota staircase ReStepper <ssrestepper@joshwel.co>"
NEUTERED_GITATTRIBUTES: Final[str] = (
    """# auto detect text files and perform lf normalization\n* text=auto\n"""
)
GH_ACT: Final[bool] = getenv("GITHUB_ACTIONS", "").lower() == "true"
GH_TOKEN: Final[str] = getenv("SS_RESTEPPER_TOKEN", "")
GH_USERNAME: Final[str] = "markjoshwel"
if GH_ACT and GH_TOKEN == "":
    print(
        "error: no personal access token found in SS_RESTEPPER_TOKEN, "
        "may not have permission to push to github"
    )
    exit(1)
_WORKERS = getenv("SS_WORKERS", None)
CPU_COUNT = cpu_count()
WORKERS: Final[int] = (
    int(_WORKERS)
    if ((_WORKERS is not None) and (_WORKERS.isdigit()))
    else 1
    if (CPU_COUNT is None)
    else CPU_COUNT
)
SIDESTEPPER_PRIMARY_LINK: Final[str] = (
    "https://forge.joshwel.co/api/v1/repos/mark/sidestepper/releases/latest"
)
SIDESTEPPER_SECONDARY_LINK: Final[str] = (
    "https://api.github.com/repos/markjoshwel/sidestepper/releases/latest"
)
SIDESTEPPER_GLOBAL_BINARY_PATH: Final[Path] = (
    Path()
    .home()
    .joinpath(
        ".local/bin/sidestepper.exe"
        if (system().lower() == "windows")
        else ".local/bin/sidestepper"
    )
)


# dictionary to share state across steps
r: dict[str, str] = {}


# define these before importing third-party modules because we use them in the import check
def generate_command_failure_message(cp: CompletedProcess) -> str:
    return "\n".join(
        [
            f"\n\nerror: command '{cp.args}' failed with exit code {cp.returncode}",
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


def log_err(
    message: str | CompletedProcess,
    exception: Exception | None = None,
    condition: bool = True,
    exitcode: int | None = None,
    show_r: bool = False,
) -> None:
    if not condition:
        return

    if isinstance(message, CompletedProcess):
        print(
            generate_command_failure_message(message)
            + (
                ("\n" + indent(text=pformat(r), prefix=INDENT) + "\n") if show_r else ""
            ),
            file=stderr,
            flush=True,
        )

    elif isinstance(exception, Exception):
        print(
            f"\n\nerror: {exception}",
            f"{INDENT}{exception.__class__.__name__}: {exception}",
            indent(text=pformat(r), prefix=INDENT),
            indent(text="\n".join(format_tb(exception.__traceback__)), prefix=INDENT)
            + (
                ("\n" + indent(text=pformat(r), prefix=INDENT) + "\n") if show_r else ""
            ),
            sep="\n",
            file=stderr,
            flush=True,
        )

    else:
        print(
            f"error: {message}"
            + (
                ("\n" + indent(text=pformat(r), prefix=INDENT) + "\n") if show_r else ""
            ),
            file=stderr,
            flush=True,
        )

    if isinstance(exitcode, int):
        exit(exitcode)


def log_debug(message: str) -> None:
    if VERBOSE:
        print(f"debug: {message}", file=stderr)


def run(
    command: str | list,
    wd: Path | str | None = None,
    capture_output: bool = True,
    give_input: str | None = None,
) -> CompletedProcess[bytes]:
    """
    exception-safe-ish wrapper around subprocess.run()

    args:
        command: str | list
            the command to run
        cwd: Path | str | None = None
            the working directory
        capture_output: bool = True
            whether to capture the output
        give_input: str | None = None
            the input to give to the command

    returns: CompletedProcess
        the return object from subprocess.run()
    """

    # noinspection PyBroadException
    try:
        cp = _run(
            command,
            shell=False if isinstance(command, list) else True,
            cwd=wd,
            capture_output=capture_output,
            input=give_input.encode() if give_input else None,
        )
    except Exception as run_exc:
        log_err(f"command '{command}' failed with exception", run_exc)
        exit(-1)
    return cp


# attempt to import third-party modules
try:
    # noinspection PyUnresolvedReferences
    from tqdm import __version__ as tqdm_version
    # noinspection PyUnresolvedReferences
    from tqdm import tqdm

    _tqdm_major, _tqdm_minor, _tqdm_patch = map(int, tqdm_version.split("."))
    if not ((4, 30, 0) <= (_tqdm_major, _tqdm_minor, _tqdm_patch)):
        log_err("unsupported tqdm version", condition=True)
        exit(-1)

except ImportError as _import_exc:
    log_err("tqdm is not installed, please run 'pip install tqdm' to install it")

    # install the missing module(s)
    if input("install these with pip? y/n: ").lower() == "y":
        print("installing...", end="", flush=True)
        _cp = run([executable, "-m", "pip", "install", "tqdm"])
        if _cp.returncode != 0:
            print(generate_command_failure_message(_cp))
            exit(-1)
        print(" done", flush=True)

    # check if it was installed successfully
    _cp = run(
        [
            executable,
            "-c",
            ";".join(["import tqdm"]),
        ]
    )
    if _cp.returncode != 0:
        log_err(_cp)
        print(
            "error: post-install check failed. reverting installation...",
            end="",
            flush=True,
        )
        _cp = run([executable, "-m", "pip", "uninstall", "tqdm", "-y"])
        log_err(_cp, condition=(_cp.returncode != 0))
        print(" done", flush=True)
        exit(-1)

    # rerun the script if we're running as one
    exit(
        run(
            [executable, Path(__file__).absolute(), *argv[1:]], capture_output=False
        ).returncode
    )


class CopyHighway:
    """
    multithreaded file copying class that gives a copy2-like function
    for use with shutil.copytree(); also displays a progress bar
    """

    pool: ThreadPool
    pbar: tqdm
    total: int
    respect_ignore: bool = True

    def __init__(
        self,
        message: str,
        total: int,
    ):
        """
        multithreaded file copying class that gives a copy2-like function
        for use with shutil.copytree()

        args:
            message: str
                message to display in the progress bar
            total: int
                total number of files to copy
        """
        self.pool = ThreadPool(
            processes=WORKERS,
        )
        self.total = total
        self.pbar = tqdm(
            total=total,
            desc=message,
            unit=" files",
            leave=False,
        )

    def callback(self, a: object):
        self.pbar.update()
        return a

    def copy2(self, source: Path | str, dest: Path | str) -> None:
        """shutil.copy2()-like function for use with shutil.copytree()"""
        self.pool.apply_async(copy2, args=(source, dest), callback=self.callback)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.pool.close()
        self.pool.join()
        self.pbar.close()


def generate_time_elapsed_string(time_taken: float) -> str:
    """generates a human-readable time-elapsed string from a time-taken float"""
    hours = int(time_taken // 3600)
    minutes = int(time_taken % 3600 // 60)
    seconds = time_taken % 60
    if time_taken > 3600:
        return f"{hours}h {minutes}′ {seconds:.1f}″"
    elif time_taken > 60:
        return f"{minutes}′ {seconds:.3f}″"
    else:
        return f"{time_taken:.3f}″"


def _sidestepper_resolve_binary_name() -> str:
    os: str
    match system().lower():
        case "windows":
            os = "windows"
        case "linux":
            os = "linux"
        case "darwin":
            os = "macos"
        case _:
            os = "unknown"

    arch: str
    match machine().lower():
        case "x86_64":
            arch = "x86_64"
        case "amd64":
            arch = "x86_64"
        case "aarch64":
            arch = "arm64"
        case _:
            arch = "unknown"

    match (os, arch):
        case ("unknown", _):
            return ""
        case (_, "unknown"):
            return ""
        case _:
            if os == "windows":
                return f"sidestepper-{os}-{arch}.exe"
            return f"sidestepper-{os}-{arch}"


def _sidestepper_resolve_binary_path(root: Path | None) -> Path | str:
    """returns path if found, empty string if not found, error message if error"""

    sidestepper_binary_name = _sidestepper_resolve_binary_name()
    if sidestepper_binary_name == "":
        return "could not determine sidestepper binary name, your platform is probably unsupported"

    if root:
        for possible_sidestepper in (
            root.joinpath(sidestepper_binary_name),
            root.joinpath(SIDESTEPPER_GLOBAL_BINARY_PATH.name),
            root.joinpath(f"Tooling/{sidestepper_binary_name}"),
            root.joinpath(f"Tooling/{SIDESTEPPER_GLOBAL_BINARY_PATH.name}"),
        ):
            log_debug(
                f"_sidestepper_resolve_binary_path: trying to use '{possible_sidestepper}'"
            )
            if not possible_sidestepper.exists():
                continue
            if not possible_sidestepper.is_file():
                return f"'{possible_sidestepper}' is not a file, this should not happen"
            if not access(possible_sidestepper, X_OK):
                return f"'{possible_sidestepper}' is not executable, this should not happen"
            return possible_sidestepper

    log_debug(
        f"_sidestepper_resolve_binary_path: trying to use '{SIDESTEPPER_GLOBAL_BINARY_PATH}'"
    )
    if not SIDESTEPPER_GLOBAL_BINARY_PATH.exists():
        return ""
    if not SIDESTEPPER_GLOBAL_BINARY_PATH.is_file():
        return f"'{SIDESTEPPER_GLOBAL_BINARY_PATH}' exists but is not a file, this should not happen"
    if not access(SIDESTEPPER_GLOBAL_BINARY_PATH, X_OK):
        return "'{SIDESTEPPER_GLOBAL_BINARY_PATH}' exists but is not executable, this should not happen"
    return SIDESTEPPER_GLOBAL_BINARY_PATH


def _sidestepper_resolve_version_file() -> Path | str:
    """
    creates parent directories if needed but not the file itself,
    returns the path if found, error message if error
    """
    match system().lower():
        case "windows":
            version_path = (
                Path()
                .home()
                .joinpath(
                    "AppData/Roaming/sota staircase/sidestepper/sidestepper.version.txt"
                )
            )
        case "linux":
            version_path = (
                Path()
                .home()
                .joinpath(".local/share/sotastaircase/sidestepper.version.txt")
            )
        case "darwin":
            version_path = (
                Path()
                .home()
                .joinpath(
                    "Library/Application Support/co.joshwel.sotastaircase/sidestepper/sidestepper.version.txt"
                )
            )
        case _:
            version_path = (
                Path()
                .home()
                .joinpath(".local/share/sotastaircase/sidestepper.version.txt")
            )

    if not version_path.exists():
        try:
            version_path.parent.mkdir(parents=True, exist_ok=True)
        except Exception as e:
            return f"could not create directory '{version_path.parent}' ({e.__class__.__name__}: {e})"

    return version_path


def _sidestepper_download_latest() -> str:
    """used to download to the global binary path, returns an error message if error"""

    # _sidestepper_resolve_binary_path() was called before this,
    # so we know the parent directories exist
    sidestepper_binary_name = _sidestepper_resolve_binary_name()
    if sidestepper_binary_name == "":
        return "could not determine sidestepper binary name, your platform is probably unsupported"
    sidestepper_version_path = _sidestepper_resolve_version_file()
    if isinstance(sidestepper_version_path, str):
        return sidestepper_version_path

    version_tag: str = ""
    download_url: str = ""

    # forge (gitea) and github have basically the same api
    for name, link in (
        ("primary", SIDESTEPPER_PRIMARY_LINK),
        ("secondary", SIDESTEPPER_SECONDARY_LINK),
    ):
        try:
            with urlopen(link) as response_json:
                response_json = json_loads(response_json.read().decode("utf-8"))
                version_tag = response_json["tag_name"]
                for asset in response_json["assets"]:
                    if asset["name"].lower() == sidestepper_binary_name.lower():
                        download_url = asset["browser_download_url"]
                        break
                else:
                    continue
                log_debug(
                    f"_sidestepper_download_latest: retrieval successful; using {name} api endpoint {link}"
                )
                break
        except Exception as e:
            print(
                f"warning: could not fetch latest sidestepper release from {name} api endpoint {link} ({e.__class__.__name__}: {e})",
                file=stderr,
            )

    if (not version_tag) or (not download_url):
        return "could not fetch latest sidestepper release from any api endpoint"

    # check if we already have the latest sidestepper release
    if sidestepper_version_path.exists():
        current_version_tag = sidestepper_version_path.read_text(
            encoding="utf-8"
        ).strip()
        if (
            (current_version_tag == version_tag)
            and (SIDESTEPPER_GLOBAL_BINARY_PATH.exists())
            and (access(SIDESTEPPER_GLOBAL_BINARY_PATH, X_OK))
        ):
            log_debug("_sidestepper_download_latest: nothing to do, returning early")
            return ""

    # download the latest sidestepper release
    log_debug(
        f"_sidestepper_download_latest: downloading latest release from {download_url}"
    )
    try:
        SIDESTEPPER_GLOBAL_BINARY_PATH.parent.mkdir(parents=True, exist_ok=True)
        with urlopen(download_url) as response:
            SIDESTEPPER_GLOBAL_BINARY_PATH.write_bytes(response.read())
    except Exception as e:
        return f"could not download latest sidestepper release {version_tag} from {download_url} to {SIDESTEPPER_GLOBAL_BINARY_PATH} ({e.__class__.__name__}: {e})"

    # make it executable (non-windows)
    if not (system().lower() == "windows"):
        try:
            SIDESTEPPER_GLOBAL_BINARY_PATH.chmod(0o755)
        except Exception as e:
            return (
                f"could not make latest sidestepper release {version_tag} located at {SIDESTEPPER_GLOBAL_BINARY_PATH} executable ({e.__class__.__name__}: {e}) "
                f"- try running `chmod +x '{SIDESTEPPER_GLOBAL_BINARY_PATH}'`"
            )

    # write the latest sidestepper version to the version file
    log_debug(
        f"_sidestepper_download_latest: writing latest sidestepper version {version_tag} to '{sidestepper_version_path}'"
    )
    try:
        sidestepper_version_path.parent.mkdir(parents=True, exist_ok=True)
        with open(sidestepper_version_path, "w") as version_file:
            version_file.write(version_tag)
    except Exception as e:
        return f"could not write latest sidestepper version {version_tag} to '{sidestepper_version_path}' ({e.__class__.__name__}: {e})"

    return ""


def sidestepper_get(root: Path | None) -> Path | str:
    """returns a path if successfully retrieved, empty string if not found, error message if error"""

    sidestepper: Path | str = _sidestepper_resolve_binary_path(root)
    log_debug(f"sidestepper_get: resolved binary path: '{sidestepper}'")

    if isinstance(sidestepper, Path):
        if root and (str(root.absolute()) in str(sidestepper.absolute())):
            # we are using a local sidestepper binary (in the repo)
            return sidestepper

        # we are using a global sidestepper binary (in ~/.local/bin)
        # let's try to update it
        log_debug("sidestepper_get: attempting update")
        dl_err = _sidestepper_download_latest()
        if dl_err:
            print(
                f"warning: tried to update sidestepper but failed, sweeping under the rug and continuing ({dl_err})",
                file=stderr,
            )

        return sidestepper

    # _sidestepper_resolve_binary_path returned an error string,
    # and it is not empty, propagate it up to the caller to print
    if isinstance(sidestepper, str) and sidestepper:
        return sidestepper

    # so let's download the latest sidestepper binary
    log_debug("sidestepper_get: downloading latest")
    dl_err = _sidestepper_download_latest()
    if dl_err:
        return dl_err
    return SIDESTEPPER_GLOBAL_BINARY_PATH


def get_root() -> Path | None:
    """finds the root of the git repository, returns None if not found"""
    root: Path = Path().cwd().resolve()
    while not (root.joinpath(".git").exists()):
        root = root.parent
        if root == Path("/").resolve():
            return None
    return root


R = TypeVar("R")


def _default_post_func(cp: R) -> R:
    """
    default post-call function for steps; does nothing

    for steps that return a CompletedProcess, this function will run the
    `_command_post_func` function

    args:
        cp: R
            return object from a step function

    returns: R
        the return object from the step function
    """
    if isinstance(cp, CompletedProcess):
        _command_post_func(cp)
    return cp


def _command_post_func(
    cp: CompletedProcess,
    fail_on_error: bool = True,
    quit_early: bool = False,
    quit_message: str = "the command gave unexpected output",
) -> CompletedProcess:
    """
    default post-call function for command steps; checks if the command was
    successful and prints the output if it wasn't

    if the command was successful, the stdout and stderr are stored in the
    shared state dictionary r under 'stdout' and 'stderr' respectively

    args:
        cp: CompletedProcess
            return object from subprocess.run()
        fail_on_error: bool
            whether to fail on error
        quit_early: bool
            whether to quit early
        quit_message: str
            the message to print if quitting early

    returns: CompletedProcess
        the return object from subprocess.run()
    """

    if quit_early:
        print(f"\n\nfailure: {quit_message}\n")

    else:
        r["stdout"] = cp.stdout.decode() if isinstance(cp.stdout, bytes) else "\0"
        r["stderr"] = cp.stderr.decode() if isinstance(cp.stderr, bytes) else "\0"
        r["blank/stdout"] = "yes" if (r["stdout"].strip() == "") else ""
        r["blank/stderr"] = "yes" if (r["stderr"].strip() == "") else ""
        r["blank"] = "yes" if (r["blank/stdout"] and r["blank/stderr"]) else ""
        r["errored"] = "" if (cp.returncode == 0) else str(cp.returncode)

        # return if the command was successful
        # or if we're not failing on error
        if (cp.returncode == 0) or (not fail_on_error):
            return cp
        else:
            print(generate_command_failure_message(cp))

    exit(
        cp.returncode if (isinstance(cp.returncode, int) and cp.returncode != 0) else 1
    )


def step(
    func: Callable[[], R],
    desc: str = "",
    post_func: Callable[[R], R] = _default_post_func,
    post_print: bool = True,
    reprint: bool = False,
    exitcode: int = -1,
) -> R:
    """
    helper function for running steps

    args:
        desc: str
            description of the step
        func: Callable[[], R] = ""
            function to run
        post_func: Callable[[R], R] = _default_post_func
            post-function to run after func
        post_print: bool = True
            whether to print done after the step
        reprint: bool = False
            whether to reprint description when done,
            for progress bars that override line output

    returns:
        R
            return object from func
    """

    # run the function
    if desc != "":
        print(f"{desc}..", end="", flush=True)

    start_time = time()

    try:
        result = func()

    except Exception as exc:
        print(
            f"\n\nfailure running step: {exc} ({exc.__class__.__name__})",
            "\n".join(format_tb(exc.__traceback__)) + "\n",
            sep="\n",
        )
        exit(exitcode)

    if desc != "" and not reprint:
        print(".", end="", flush=True)

    # run the post-function
    try:
        rp = post_func(result)

    except Exception as exc:
        print(
            f"\n\nfailure running post-step: {exc} ({exc.__class__.__name__})",
            "\n".join(format_tb(exc.__traceback__)) + "\n",
            sep="\n",
        )
        exit(exitcode)

    end_time = time()

    # yay
    if desc != "" and post_print:
        print(
            (f"{desc}..." if reprint else "")
            + f" done in {generate_time_elapsed_string(end_time - start_time)}",
            flush=True,
        )

    return rp


def post_remote_v(cp: CompletedProcess) -> CompletedProcess:
    """
    post-call function for 'git remote -v' command, parses the output and
    checks for the forge and github remotes, storing them in the shared state
    under 'remote/forge', 'remote/forge/url', 'remote/github', and
    'remote/github/url' respectively
    """

    if not isinstance(cp.stdout, bytes):
        return _command_post_func(cp)

    for line in cp.stdout.decode().split("\n"):
        # github  https://github.com/markjoshwel/sota (fetch)
        # github  https://github.com/markjoshwel/sota (push)
        # origin  https://forge.joshwel.co/mark/sota.git (fetch)
        # origin  https://forge.joshwel.co/mark/sota.git (push)

        split_line = line.split(maxsplit=1)
        if len(line) < 2:
            continue

        # remote='origin'  url='https://forge.joshwel.co/mark/sota.git (fetch)'
        remote, url = split_line

        # clean up the url
        if (REPO_URL_FORGE in url) or (REPO_URL_GITHUB in url):
            # url='https://forge.joshwel.co/mark/sota.git'
            url = url.split("(", maxsplit=1)[0].strip()

        if REPO_URL_FORGE in url:
            r["remote/forge"] = remote
            r["remote/forge/url"] = url

        elif REPO_URL_GITHUB in url:
            r["remote/github"] = remote
            r["remote/github/url"] = url

    return _command_post_func(cp)


def post_filter_repo_check(cp: CompletedProcess) -> CompletedProcess:
    """
    post-call function for checking if git-filter-repo is installed
    and optionally installing it if it isn't
    """

    if cp.returncode == 0:
        return cp

    if input("git filter-repo is not installed, install it? y/n: ").lower() != "y":
        print(
            "install it using 'pip install git-filter-repo' "
            "or 'pipx install git-filter-repo'",
        )
        return cp

    # check if pipx is installed
    use_pipx = False

    check_pipx_cp = run(["pipx", "--version"])
    if check_pipx_cp.returncode == 0:
        use_pipx = True

    # install git-filter-repo
    pip_invocation: list[str] = ["pipx"] if use_pipx else [executable, "-m", "pip"]
    print(
        f"running '{' '.join([*pip_invocation, 'install', 'git-filter-repo'])}'... ",
        end="",
        flush=True,
    )
    install_rc = run([*pip_invocation, "install", "git-filter-repo"])
    if install_rc.returncode != 0:
        print("error")
        _command_post_func(install_rc)
        exit(install_rc.returncode)
    else:
        print("done\n")

    # check if it is reachable
    if run(["git", "filter-repo", "--version"]).returncode != 0:
        # revert
        run([*pip_invocation, "uninstall", "git-filter-repo"])
        print(
            "failure: could not install git-filter-repo automatically. "
            "do it yourself o(*≧▽≦)ツ┏━┓"
        )
        exit(-1)

    return cp


def main() -> None:
    repo_path = get_root()
    log_err(
        "could not find a git repository in the working or parent directories",
        condition=not isinstance(repo_path, Path),
        exitcode=1,
    )
    assert isinstance(repo_path, Path)

    sidestepper_binary = sidestepper_get(repo_path)
    log_err(
        "could not find sidestepper binary",
        condition=(sidestepper_binary == ""),
        exitcode=2,
    )
    log_err(
        sidestepper_binary,  # type: ignore
        condition=isinstance(sidestepper_binary, str),
        exitcode=3,
    )
    log_debug(f"sidestepper binary is '{sidestepper_binary}'")
    assert isinstance(sidestepper_binary, Path)

    cumulative_start_time = time()
    with TemporaryDirectory(delete="--keep" not in argv) as temp_dir:
        temp_path = Path(temp_dir)
        print(
            "sota staircase ReStepper v12",
            f"     real repo : {repo_path}",
            f"     temp repo : {temp_dir}",
            f"   sidestepper : {sidestepper_binary}"
            + (f"\n     is gh act : {GH_ACT}\n" if GH_ACT else "\n"),
            sep="\n",
        )

        # helper partial function for command
        def cmd(
            command: str,
            wd: Path | str = temp_dir,
            capture_output: bool = True,
            give_input: str | None = None,
        ) -> Callable[[], CompletedProcess]:
            return lambda: run(
                command,
                wd=wd,
                capture_output=capture_output,
                give_input=give_input,
            )

        log_debug("checking for git filter-repo")
        step(
            func=cmd("git filter-repo --version"),
            post_func=post_filter_repo_check,
            exitcode=4,
        )

        log_debug("checking git status")
        step(func=cmd("git status --porcelain", wd=repo_path), exitcode=5)
        if (not r["blank"]) and ("--idonotcare" not in argv):
            log_err(
                "repository is not clean, please commit changes first",
                show_r=True,
                exitcode=5,
            )

        step(
            desc="1 pre | finding large files",
            func=cmd(f"{sidestepper_binary}", wd=repo_path),
            exitcode=6,
        )
        if r["errored"]:
            log_err(
                "could not find large files",
                show_r=True,
                exitcode=6,
            )

        def get_repo_files() -> list[Path]:
            files: list[Path] = []
            for file in run("git ls-files", wd=repo_path).stdout.decode().splitlines():
                file = file.strip()
                if not file:
                    continue
                files.append(Path(file))
            for src in repo_path.joinpath(".git").rglob("*"):
                files.append(Path(src).relative_to(repo_path))
            # no .sotaignore because git filter-repo might remove
            # it if it wasn't already in history
            return files

        repo_files = step(
            desc="2 pre | scanning repo",
            func=lambda: get_repo_files(),
            exitcode=7,
        )

        def duplicate_repo():
            with CopyHighway(
                message="3 pre | duplicating repo",
                total=len(repo_files),
            ) as copier:
                for src in repo_files:
                    dst = temp_path.joinpath(src)
                    if not dst.parent.exists():
                        dst.parent.mkdir(parents=True, exist_ok=True)
                    copier.copy2(repo_path.joinpath(src), dst)

        step(
            desc="3 pre | duplicate repo",
            func=lambda: duplicate_repo(),
            reprint=True,
            exitcode=8,
        )

        log_debug("double-checking pathlib.Path.cwd()")
        step(
            func=cmd(
                'python -c "import pathlib; print(pathlib.Path.cwd().absolute())"'
            ),
            exitcode=9,
        )
        if str(temp_path.absolute()) != r["stdout"].strip():
            log_err(
                f"not inside the temp dir '{str(temp_path.absolute())}' (whuh? internal?)",
                show_r=True,
                exitcode=9,
            )

        log_debug("checking for forge and github remotes")
        step(
            func=cmd("git remote -v"),
            post_func=post_remote_v,
            exitcode=10,
        )
        if "remote/forge" not in r:
            log_err(
                "no forge remote found (whuh?)",
                show_r=True,
                exitcode=10,
            )

        log_debug("getting the current branch")
        step(func=cmd("git branch --show-current"), exitcode=11)
        branch = r["stdout"].strip()
        if r.get("errored", "yes") or branch == "":
            log_err("couldn't get current branch (whuh?)", show_r=True, exitcode=11)

        log_debug("checking if up to date with forge: fetching")
        step(func=cmd(f"git fetch {r['remote/forge']}"), exitcode=12)

        log_debug(
            f"checking if up to date with forge: rev-list HEAD...{r['remote/forge']}/{branch} count"
        )
        step(
            func=cmd(f"git rev-list HEAD...{r['remote/forge']}/{branch} --count"),
            exitcode=13,
        )
        if (r.get("stdout", "").strip() != "0") and ("--idonotcare" not in argv):
            log_err(
                "not up to date with forge... sync your changes first? (whuh?)",
                show_r=True,
                exitcode=13,
            )

        step(desc="4 lfs | fetch lfs objects", func=cmd("git lfs fetch"), exitcode=14)

        step(
            desc="5 lfs | migrating lfs objects",
            func=cmd(
                f'git lfs migrate export --everything --include="*" --remote={r["remote/forge"]} --yes',
                # give_input="y\n",
            ),
            exitcode=15,
        )

        step(
            desc="6 lfs | uninstall lfs in repo",
            func=cmd("git lfs uninstall"),
            exitcode=16,
        )

        log_debug("checking if lfs objects still exist")
        step(
            func=cmd("git lfs ls-files"),
            exitcode=17,
        )
        if not r["blank"]:
            log_err(
                "critical error (whuh? internal?): "
                "lfs objects still exist post-migrate and uninstall",
                show_r=True,
                exitcode=18,
            )

        if (repo_sotaignore_path := repo_path.joinpath(".sotaignore")).exists():
            log_debug(".sotaignore exists, reading")

            sotaignore: str = ""
            try:
                sotaignore = repo_sotaignore_path.read_text(encoding="utf-8").strip()
            except Exception as exc:
                log_err(
                    "couldn't read .sotaignore file",
                    exception=exc,
                    exitcode=19,
                )

            sotaignored_files: list[str] = [
                line
                for line in sotaignore.splitlines()
                if not line.startswith("#") and line.strip() != ""
            ]
            log_debug(f"found {len(sotaignored_files)} file(s) in .sotaignore")

            step(
                desc=f"7 lfs | filter repo and {len(sotaignored_files)} file(s)",
                func=cmd(
                    "git filter-repo --force --strip-blobs-bigger-than 100M --invert-paths "
                    + " ".join(f'--path "{lf}"' for lf in sotaignored_files)
                ),
                exitcode=20,
            )

            log_debug(
                "copying .sotaignore to temp repo; step 5 wipes uncommitted changes"
            )
            step(
                func=lambda: copy2(
                    repo_sotaignore_path, Path(temp_path).joinpath(".sotaignore")
                ),
                exitcode=21,
            )

        def neuter_and_commit() -> CompletedProcess:
            # neuter
            for repo_file in temp_path.rglob(".gitattributes"):
                repo_file.write_text(NEUTERED_GITATTRIBUTES, encoding="utf-8")

            # add
            if GH_ACT:
                cp = cmd("git config user.name 'github-actions[bot]'")()
                if cp.returncode != 0:
                    return cp

                cp = cmd(
                    "git config user.email 'github-actions[bot]@users.noreply.github.com'"
                )()
                if cp.returncode != 0:
                    return cp

            cp = cmd("git add -A")()
            if cp.returncode != 0:
                return cp

            # and commit
            return cmd(
                f'git commit --allow-empty -am "{COMMIT_MESSAGE}" --author="{COMMIT_AUTHOR}"',
            )()

        step(
            desc="8 fin | neuter .gitattributes and commit",
            func=neuter_and_commit,
            exitcode=22,
        )

        if r.get("remote/github") is None:
            log_debug(f"github remote doesn't exist, adding via {REPO_URL_GITHUB=}")
            step(
                func=cmd(f"git remote add github https://{REPO_URL_GITHUB}.git"),
                exitcode=23,
            )
            if r.get("errored", "yes"):
                log_err(
                    "couldn't add github remote",
                    show_r=True,
                    exitcode=23,
                )
            r["remote/github"] = "github"

        step(
            desc=f"9 fin | fetch {r['remote/github']}",
            func=cmd(f"git fetch {r['remote/github']}"),
            exitcode=24,
        )

        push_invocation = (
            f"git push {r['remote/github']} {branch} --force"
            if not GH_ACT
            else f"git push https://{GH_USERNAME}:{GH_TOKEN}@{REPO_URL_GITHUB}.git {branch} --force"
        )

        step(
            desc=f"X fin | pushing to {r['remote/github']}/{branch}",
            func=cmd(push_invocation if ("--test" not in argv) else "git --version"),
            exitcode=25,
        )

    cumulative_end_time = time()
    print(
        f"\n--- done! took {generate_time_elapsed_string(cumulative_end_time - cumulative_start_time)}~ "
        "☆*: .｡. o(≧▽≦)o .｡.:*☆ ---",
        flush=True,
    )


if __name__ == "__main__":
    main()
