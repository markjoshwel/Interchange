# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "mcstatus",
# ]
# ///

# erstellen: a minecraft server.jar runner with stdin passthrough and
# on-startup command execution
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

from io import TextIOWrapper
from threading import Thread
from subprocess import Popen, PIPE
from time import sleep, time
from typing import IO
from mcstatus import JavaServer
from os import getenv
from sys import argv, stderr, stdout, stdin
from pathlib import Path


JAVA_COMMAND: list[str] = [
    "java",
    "-Xms8G",
    "-Xmx8G",
    "-XX:+UseG1GC",
    "-XX:+ParallelRefProcEnabled",
    "-XX:MaxGCPauseMillis=200",
    "-XX:+UnlockExperimentalVMOptions",
    "-XX:+DisableExplicitGC",
    "-XX:+AlwaysPreTouch",
    "-XX:G1NewSizePercent=30",
    "-XX:G1MaxNewSizePercent=40",
    "-XX:G1HeapRegionSize=8M",
    "-XX:G1ReservePercent=20",
    "-XX:G1HeapWastePercent=5",
    "-XX:G1MixedGCCountTarget=4",
    "-XX:InitiatingHeapOccupancyPercent=15",
    "-XX:G1MixedGCLiveThresholdPercent=90",
    "-XX:G1RSetUpdatingPauseTimePercent=5",
    "-XX:SurvivorRatio=32",
    "-XX:+PerfDisableSharedMem",
    "-XX:MaxTenuringThreshold=1",
    # "-jar",
    # SERVER_JAR_FILE,
    # "nogui",
]


def stop_server(server: Popen[bytes]) -> int:
    if (retcode := server.poll()) is None:
        try:
            # write stop command
            assert server.stdin is not None
            server.stdin.write(b"stop\n")
            server.stdin.flush()
            # wait up to 60 seconds
            server.wait(timeout=60)
        except (Exception, AssertionError):
            # worse comes to worst, just terminate
            server.terminate()
        return 1
    else:
        return retcode


def wait_for_server(server: JavaServer, timeout: int = 60) -> bool:
    start_time = time()
    while (time() - start_time) < timeout:
        try:
            server.status()
            return True
        except KeyboardInterrupt:
            return False
        except Exception:
            sleep(1)
    return False


def output_reader(stream: IO[bytes] | None, is_stderr: bool = False) -> None:
    if stream is None:
        return
    for line in iter(stream.readline, b""):
        print(line.decode(), file=stderr if is_stderr else stdout, end="")


def start_server(server_jar: Path) -> Popen[bytes]:
    server_process = Popen(
        [*JAVA_COMMAND, "-jar", server_jar.absolute(), "nogui"],
        cwd=server_jar.absolute().parent,
        stdin=PIPE,
        stdout=PIPE,
        stderr=PIPE,
    )

    # read in from the servers stderr and stdout
    # and pipe/reprint to the respective streams
    stdout_thread = Thread(
        target=output_reader,
        args=(server_process.stdout, False),
        daemon=True,
    )
    stderr_thread = Thread(
        target=output_reader,
        args=(server_process.stderr, True),
        daemon=True,
    )
    stdout_thread.start()
    stderr_thread.start()

    return server_process


def print_into_server(_message: str | bytes, server: Popen[bytes]) -> bool:
    message: bytes = b""
    if isinstance(_message, bytes):
        message = _message
    elif isinstance(_message, str):
        message = _message.encode()
    else:
        message = str(_message).encode()

    if not message.endswith(b"\n"):
        message += b"\n"

    if server.stdin is None:
        return False

    server.stdin.write(message)
    server.stdin.flush()
    return True


def main() -> None:
    # 0. resolve server ip and jar path
    server_ip = "localhost"
    if server_ip_env := getenv("ERSTELLEN_TARGET_IP"):
        server_ip = server_ip_env

    server_jar: Path = (
        Path.cwd().joinpath("server.jar") if len(argv) < 2 else Path(argv[1])
    )
    if not server_jar.exists():
        print(
            f"erstellen: error: {server_jar} not found in working directory",
            file=stderr,
        )
        exit(1)

    # 1. start server
    server_dir: Path = server_jar.absolute().parent
    print(f"erstellen: starting '{server_jar}'...", file=stderr)
    server_process = start_server(server_jar)

    # 2. wait for server to start
    print("erstellen: waiting for server to start...", file=stderr)
    server = JavaServer(server_ip)
    if not wait_for_server(server):
        print("erstellen: error: server failed to start in time", file=stderr)
        stop_server(server_process)
        exit(1)

    # 3. run commands in erstellen.commands.txt
    print("erstellen: server is up!", file=stderr)
    try:
        sleep(3)
    except KeyboardInterrupt:
        pass
    erstellen_commands_path = server_dir.joinpath("erstellen.commands.txt")
    erstellen_commands: list[str] = []
    if erstellen_commands_path.exists():
        erstellen_commands = erstellen_commands_path.read_text(
            encoding="utf-8"
        ).splitlines()

    for command in erstellen_commands:
        print(f"erstellen: running '{command}'...", file=stderr)
        if not print_into_server(command, server_process):
            print(
                f"erstellen: error: server process stdin is None, command '{command}' not executed",
                file=stderr,
            )
        else:
            print(f"erstellen: '{command}'", file=stderr)

    # 4. run server until Ctrl+C with stdin passthrough
    try:
        if server_process.stdin is None:
            print(
                "erstellen: error: server process stdin is None, stdin passthrough disabled",
                file=stderr,
            )
            while True:
                pass
        elif not isinstance(stdin, TextIOWrapper):
            print(
                "erstellen: error: system stdin is None, stdin passthrough disabled",
                file=stderr,
            )
            while True:
                pass
        else:
            for line in stdin:
                if server_process.poll() is None:
                    print_into_server(line, server_process)

    except KeyboardInterrupt:
        pass

    # 5. stop
    exit(stop_server(server_process))


if __name__ == "__main__":
    main()
