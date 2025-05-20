# GirlsLastStickers: booru scraper + image processor for telegram-ready girls last tour stickers
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

# %% setup

from datetime import datetime, timedelta
from functools import wraps
from pathlib import Path
from shutil import which
from subprocess import run
from time import sleep
from typing import Callable, Final, ParamSpec, TypeVar
from urllib.parse import urlsplit

from bs4 import BeautifulSoup
from requests import Response, get

# constants
USER_AGENT: Final[str] = (
    "Mozilla/5.0 (compatible; lfcircle; https://github.com/markjoshwel/lfcircle)"
)
BOORU_SEARCH: Final[list[str]] = [
    "https://safebooru.org/index.php?page=post&s=list&tags=tsukumizu_yuu+lowres"
]
BOORU_PREFIX: Final[str] = "https://safebooru.org"
BOORU_POST_NEEDLE_HREF_SUBSTRING = "/index.php?page=post&s=view&id="

P = ParamSpec("P")
R = TypeVar("R")


class Limiter:
    """helper to class to not bomb websites"""

    max_per_second: int = 1
    last_call: datetime | None = None

    def limit(
        self,
        func: Callable[P, R],
        sleeper: Callable[[float], None] = sleep,
    ) -> Callable[P, R]:
        @wraps(func)
        def wrapper(*args: P.args, **kwargs: P.kwargs):
            if self.last_call is None:
                self.last_call = datetime.now()
                return func(*args, **kwargs)

            while (self.last_call + timedelta(seconds=1)) > (now := datetime.now()):
                sleeper(1)

            self.last_call = now
            return func(*args, **kwargs)

        return wrapper


# %% get pages

limiter = Limiter()
booru_search_results: dict[str, BeautifulSoup] = {}

for idx, url in enumerate(BOORU_SEARCH, start=1):
    print(f"[{datetime.now()}] getting search page {url} ({idx}/{len(BOORU_SEARCH)})")
    page: Response = limiter.limit(get)(
        url,
        headers={"User-Agent": USER_AGENT},
    )

    if page.status_code != 200:
        raise Exception(f"non-nominal status code {page.status_code} for '{url}'")

    booru_search_results[url] = BeautifulSoup(page.text, "html5lib")
else:
    print(f"gathered {len(booru_search_results)} search pages")

# %% get post from pages

post_links: list[str] = []

for search_page in booru_search_results.values():
    for tag_with_href in search_page.find_all(
        href=lambda href: href is not None and BOORU_POST_NEEDLE_HREF_SUBSTRING in href
    ):
        post_links.append(tag_with_href["href"])  # type: ignore
else:
    print(f"gathered {len(post_links)} post links")

# %% get each page

post_pages: dict[str, BeautifulSoup] = {}

for idx, url in enumerate(post_links, start=1):
    print(f"[{datetime.now()}] getting post page {url} ({idx}/{len(post_links)})")
    page = limiter.limit(get)(
        f"{BOORU_PREFIX}{url}",
        headers={"User-Agent": USER_AGENT},
    )

    if page.status_code != 200:
        raise Exception(f"non-nominal status code {page.status_code} for '{url}'")

    post_pages[url] = BeautifulSoup(page.text, "html5lib")
else:
    print(f"gathered {len(post_pages)} post pages")

# %% get a download link from each post

image_links: list[str] = []

for post_page in post_pages.values():
    for tag_with_href in post_page.find_all(href=True):
        if "Original image" in tag_with_href.text:
            image_links.append(tag_with_href["href"])  # type: ignore
else:
    print(f"gathered {len(image_links)} image links")

# %% download each image

(images_dir := Path.cwd().joinpath("images")).mkdir(parents=True, exist_ok=True)
print(f"making images dir at {images_dir}")

images_paths: list[Path] = []
for idx, url in enumerate(image_links, start=1):
    image_filename = Path(urlsplit(url).path).name
    image_download_path = images_dir.joinpath(image_filename)

    # so that i dont have to redownload every damn time
    if image_download_path.exists():
        images_paths.append(image_download_path)
        continue

    print(f"[{datetime.now()}] downloading image {url} ({idx}/{len(image_links)})")
    image: Response = limiter.limit(get)(
        url,
        headers={"User-Agent": USER_AGENT},
    )

    if image.status_code != 200:
        raise Exception(f"non-nominal status code {image.status_code} for '{url}'")

    image_download_path.write_bytes(image.content)
    images_paths.append(image_download_path)

else:
    print(f"downloaded {len(images_paths)} images")

# %% turn the images into telegram-friendly stickers

vipsthumbnail_invocation: list[str] = ["vipsthumbnail"]

if which("vipsthumbnail") is None:
    if which("nix") is not None:
        vipsthumbnail_invocation = [
            "nix",
            "shell",
            "nixpkgs#vips",
            "-c",
            "vipsthumbnail",
        ]
    else:
        raise EnvironmentError(
            "neither `vipsthumbnail` nor the `nix` command are available, go download vips!"
        )

(stickers_dir := Path.cwd().joinpath("stickers")).mkdir(parents=True, exist_ok=True)
print(f"making stickers dir at {stickers_dir}")

for idx, original_image_path in enumerate(images_paths, start=1):
    processed_image_path = stickers_dir.joinpath(
        original_image_path.stem + ".sticker.png"
    )
    if processed_image_path.exists():
        continue

    full_invocation = vipsthumbnail_invocation + [
        str(original_image_path.absolute()),
        "--size",
        "512",
        f"--output={str(processed_image_path).replace(' ', '\\ ')}",
    ]

    print(f"({idx}/{len(images_paths)}) -> `{' '.join(full_invocation).strip()}`")
    cp = run(full_invocation, check=True)
else:
    print(f"generated {idx} stickers")
