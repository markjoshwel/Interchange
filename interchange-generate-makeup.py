from tomlkit import loads as loads_toml, dumps as dumps_toml, inline_table  # pyright: ignore[reportUnknownVariableType]
from json import loads as loads_json
from pathlib import Path
from typing import cast, TypedDict
from subprocess import run
from tempfile import TemporaryDirectory
from traceback import print_tb

interchange_path = Path(__file__).parent.joinpath("interchange.toml")
interchange = loads_toml(interchange_path.read_text())
git_forges: list[str] = [
    "github.com",
    "forge.joshwel.co",
    "git.sr.ht",
    "sr.ht",
]


class EnryLanguageResult(TypedDict):
    color: str
    language: str
    percentage: float
    size: int


class ProjectEnryData(TypedDict):
    subdirectory: str
    branch: str
    result: list[EnryLanguageResult]


class InterchangeProject(TypedDict):
    name: str
    description: str
    urls: list[str]  # "<url>" or "<description>: <url>"
    tags: list[str]
    enry: ProjectEnryData


projects = cast(
    dict[str, InterchangeProject],
    interchange.get("interchange", {"projects": {}}).get("projects", {}),  # pyright: ignore[reportUnknownMemberType]
)

with TemporaryDirectory(ignore_cleanup_errors=True) as tmpdir:
    for p_name, p_info in projects.items():
        print(f"[interchange.project.{p_name}]")

        if "urls" not in p_info:
            print(f"error: project '{p_name}' is missing 'urls'")
            continue

        p_urls = p_info["urls"]
        if not isinstance(p_urls, list):  # pyright: ignore[reportUnnecessaryIsInstance]
            print(f"error: project '{p_name}' has invalid 'urls'")
            continue

        elif not all(isinstance(url, str) for url in p_info["urls"]):  # pyright: ignore[reportUnnecessaryIsInstance]
            print(
                f"error: project '{p_name}' has invalid (non-string) elements in 'urls'"
            )
            continue

        target_subdir: str = ""
        target_branch: str = ""
        if "enry" in p_info:
            if "subdirectory" in p_info["enry"]:
                target_subdir = p_info["enry"]["subdirectory"]
            if "branch" in p_info["enry"]:
                target_branch = p_info["enry"]["branch"]

        for url_str in p_urls:
            sp_url = url_str.split(maxsplit=1)
            _, url = sp_url[0], sp_url[1] if len(sp_url) == 2 else sp_url[0]

            for forge in git_forges:
                if forge not in url:
                    continue

                repo_url = url
                if "/tree" in url:  # github, sourcehut
                    repo_url, _ = url.split("/tree", maxsplit=1)
                elif "/blob" in url:  # github
                    repo_url, _ = url.split("/blob", maxsplit=1)
                elif "/src" in url:  # gitea/forgejo
                    repo_url, _ = url.split("/src", maxsplit=1)

                repo_dir_name: str = repo_url.split("/")[-1].split(".git")[0]
                repo_dir = Path(tmpdir).joinpath(repo_dir_name)
                
                if not repo_dir.exists():
                    print(
                        f" -> using `git clone {repo_url} {repo_dir_name}`"
                        + (f" (subdir is '{target_subdir}')" if target_subdir else "")
                    )
                    cp_clone = run(
                        ["git", "clone", repo_url, f"{repo_dir_name}"],
                        cwd=tmpdir,
                        capture_output=True,
                    )
                    if cp_clone.returncode != 0:
                        print(
                            f"error: failed to clone project '{p_name}' from '{url}'",
                        )
                        for line in cp_clone.stderr.decode().splitlines():
                            print(
                                f" ... {line}",
                            )
                        continue
                else:
                    print(" -> skipping git clone, already pulled")
                
                # switch branch
                original_branch: str = "main"
                cp_get_branch = run(
                    ["git", "branch", "--show-current"],
                    cwd=repo_dir,
                    capture_output=True,
                )
                original_branch = cp_get_branch.stdout.decode().strip()
                
                if cp_get_branch.returncode != 0:
                    print(f"error: failed to get current branch for repo '{repo_dir}'")
                    for line in cp_get_branch.stderr.decode().splitlines():
                        print(f" ... {line}")
                    continue
                
                if target_branch:
                    print(f" -> switching to branch '{target_branch}'")
                    cp_switch_branch = run(
                        ["git", "checkout", target_branch],
                        cwd=repo_dir,
                        capture_output=True,
                    )
                    if cp_switch_branch.returncode != 0:
                        print(f"error: failed to switch to branch '{target_branch}'")
                        for line in cp_switch_branch.stderr.decode().splitlines():
                            print(f" ... {line}")
                        continue

                # figure out what languages are in the project
                # enry is a golang implementation of github's linguist

                enry_invocation: list[str] = [str(Path(__file__).parent.joinpath("enry/enry").absolute()), "--json"]
                enry_wd: Path = repo_dir.joinpath(target_subdir) if target_subdir else repo_dir
                print(f" -> running enry on {enry_wd}")
                cp_enry = run(
                    enry_invocation,
                    cwd=enry_wd,
                    capture_output=True,
                )
                if cp_enry.returncode != 0:
                    print(
                        f"error: failed to run enry on project '{p_name}' from '{url}'",
                    )
                    for line in cp_enry.stderr.decode().splitlines():
                        print(f" ... {line}")
                    continue

                print(" -> parsing enry")
                try:
                    enry_result = loads_json(cp_enry.stdout.decode())  # pyright: ignore[reportAny]

                    if "enry" not in p_info:
                        p_info["enry"] = {}
                    p_info["enry"]["result"] = []

                    for lang_results in enry_result:  # pyright: ignore[reportAny]
                        
                        assert "color" in lang_results, (
                            "enry json output should contain 'size' key"
                        )
                        
                        assert "language" in lang_results, (
                            "enry json output should contain 'size' key"
                        )
                        
                        assert "percentage" in lang_results, (
                            "enry json output should contain 'size' key"
                        )
                        
                        assert "size" in lang_results, (
                            "enry json output should contain 'size' key"
                        )

                        results = inline_table()
                        results["color"] = lang_results["color"]
                        results["language"] = lang_results["language"]
                        results["percentage"] = float(lang_results["percentage"].replace("%", ""))  # pyright: ignore[reportAny]
                        results["size"] = lang_results["size"]

                        # ungodly type cast hack
                        projects[p_name]["enry"]["result"].append(
                            cast(EnryLanguageResult, cast(object, results))
                        )

                except Exception as exc:
                    print(
                        f"error: failed to parse enry output for project '{p_name}' from '{url}' ({exc.__class__.__name__}: {exc})",
                    )
                    print_tb(exc.__traceback__)
                    continue
                
                if target_branch:
                    print(f" -> switching back to '{original_branch}'")
                    cp_switch_branch = run(
                        ["git", "checkout", original_branch],
                        cwd=repo_dir,
                        capture_output=True,
                    )
                    if cp_switch_branch.returncode != 0:
                        print(f"error: failed to switch back to branch '{original_branch}'")
                        for line in cp_switch_branch.stderr.decode().splitlines():
                            print(f" ... {line}")
                        continue

                print(" -> ok\n")

                break
            else:
                print(" -> no urls found to clone\n")


print(interchange_path.write_text(dumps_toml(interchange)))
