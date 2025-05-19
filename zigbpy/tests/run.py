# /// script
# requires-python = ">=3.12"
# dependencies = [
#     "ziglang",
# ]
# ///

from binascii import hexlify
from pathlib import Path
from platform import system
from subprocess import PIPE, CompletedProcess, Popen, run
from sys import executable as python
from tempfile import TemporaryDirectory
from typing import NamedTuple

zigbpy2 = Path(__file__).parent.parent.joinpath("zigbpy2.py")
assert (
    zigbpy2.exists()
), f"expected zigbpy2 to be at {zigbpy2.absolute()}, was not the case"


class Test(NamedTuple):
    path: Path
    pass_in: bytes
    expected_out: bytes
    expected_success: bool = True
    timeout: float | None = None

    def build_error_message(
        self,
        header: str,
        stdout: bytes | None = None,
        stderr: bytes | None = None,
        expected: bytes | None = None,
    ) -> str:
        return (
            header
            + (
                (
                    "\n... expected:"
                    f"\n... ... <{hexlify(self.expected_out, ' ').decode(errors='ignore')}>"
                    f"\n... ... (utf-8) {repr(self.expected_out.decode(errors='ignore'))}"
                    "\n... received:"
                    f"\n... ... <{hexlify(stdout, ' ').decode(errors='ignore')}>"
                    f"\n... ... as utf-8: {repr(stdout.decode(errors='ignore'))}"
                )
                if isinstance(expected, bytes) and isinstance(stdout, bytes)
                else ""
            )
            + (
                (
                    "\n... stdout:\n"
                    + "\n".join(
                        f"... ... {line}"
                        for line in stdout.strip().decode(errors="ignore").splitlines()
                    )
                )
                if isinstance(stdout, bytes) and stdout.strip()
                else ""
            )
            + (
                (
                    "\n... stderr:\n"
                    + "\n".join(
                        f"... ... {line}"
                        for line in stderr.strip().decode(errors="ignore").splitlines()
                    )
                )
                if isinstance(stderr, bytes) and stderr.strip()
                else ""
            )
        )

    def run(self, working_dir: Path) -> None:
        test_compile: CompletedProcess[bytes] = run(
            [python, zigbpy2, self.path],
            capture_output=True,
            cwd=working_dir,
        )
        if self.expected_success:
            assert test_compile.returncode == 0, self.build_error_message(
                header=f"compilation failed with code {test_compile.returncode}",
                stderr=test_compile.stderr,
                stdout=test_compile.stdout,
            )
        else:
            return

        expected_output_path: Path = working_dir.joinpath(
            f"{self.path.stem}" + (".exe" if system() == "Windows" else "")
        )
        assert expected_output_path.exists(), self.build_error_message(
            header="compilation succeeded but output file was not found",
        )

        test_run = Popen(
            [expected_output_path],
            stdin=PIPE,
            stdout=PIPE,
            stderr=PIPE,
        )

        test_stdout: bytes
        test_stderr: bytes
        (test_stdout, test_stderr) = test_run.communicate(
            input=self.pass_in,
            timeout=self.timeout,
        )

        if not (self.expected_success and test_run.returncode == 0):
            assert test_run.returncode != 0, "test is supposed to fail"
            return

        # bf only outputs to stdout
        assert test_stdout == self.expected_out, self.build_error_message(
            header="unexpected output",
            stdout=test_stdout,
            stderr=test_stderr,
            expected=self.expected_out,
        )

        assert test_run.returncode == 0, self.build_error_message(
            header="program return is non-zero",
            stdout=test_stdout,
            stderr=test_stderr,
            expected=self.expected_out if self.expected_out else None,
        )


def main() -> None:
    tests_dir = Path(__file__).parent.parent.joinpath("tests")
    tests: list[Test] = [
        Test(
            path=tests_dir.joinpath("cristofd/1-io.b"),
            pass_in=b"\x0a\x04\x04",
            expected_out=b"LK\nLK\n",
        ),
        Test(
            path=tests_dir.joinpath("cristofd/2-cells.b"),
            pass_in=b"",
            expected_out=b"#\n",
        ),
        Test(
            path=tests_dir.joinpath("cristofd/3-bounds.b"),
            pass_in=b"",
            expected_out=b"",
            expected_success=False,
        ),
        Test(
            path=tests_dir.joinpath("cristofd/4-obscure.b"),
            pass_in=b"",
            expected_out=b"H\n",
        ),
        Test(
            path=tests_dir.joinpath("cristofd/5-unmatched.b"),
            pass_in=b"",
            expected_out=b"",
            expected_success=False,
        ),
        Test(
            path=tests_dir.joinpath("cristofd/6-unmatched.b"),
            pass_in=b"",
            expected_out=b"",
            expected_success=False,
        ),
        Test(
            path=tests_dir.joinpath("cristofd/7-rot13.b"),
            pass_in=b"~mlk zyx\x04",
            expected_out=b"~zyx mlk",
            timeout=5.0,
        ),
    ]

    ok: int = 0
    with TemporaryDirectory(delete=False) as temp_dir:
        for n, test in enumerate(tests, start=1):
            print(f"running: test {n}...", end="", flush=True)
            try:
                test.run(Path(temp_dir))
            except AssertionError as e:
                print(f" failed...\ntest {n} failed by assertion: {e}\n", flush=True)
                continue
            else:
                print(" ok!", flush=True)
                ok += 1

    if ok == len(tests):
        print(f"result: okay! <3 ({ok}/{len(tests)})")
        exit(0)
    else:
        print("result: not all tests a-okay... </3 ({ok}/{len(tests)})")
        exit(1)


if __name__ == "__main__":
    main()
