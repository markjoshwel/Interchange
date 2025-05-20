// sota staircase SideStepper
// a fast .gitignore-respecting large file finder
//
// Copyright (c) 2025 mark joshwel <mark@joshwel.co>
// Zero-Clause BSD Licence
//
// Permission to use, copy, modify, and/or distribute this software for
// any purpose with or without fee is hereby granted.
//
// THE SOFTWARE IS PROVIDED “AS IS” AND THE AUTHOR DISCLAIMS ALL
// WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE
// FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY
// DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN
// AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT
// OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

use ignore;
use std::error::Error;
use std::fs::metadata;
use std::io::Write;
use std::path::{Path, PathBuf};
use std::process::exit;
use std::time::{Duration, SystemTime};
use std::{env, fs, io, path};

const SOTA_SIDESTEP_LARGE_FILE_SIZE: u64 = 100000000; // 100mb
const VERSION_OVERALL: u8 = 5;
const VERSION_IMPLEMENTATION: u8 = 3;
const VERSION_ALGORITHM: u8 = 6;

struct Behaviour {
    repo_dir_path: PathBuf,
    repo_sotaignore_path: PathBuf,
    large_file_size: u64,
    plumbing: bool,
    version: bool,
}

fn cli_get_behaviour() -> Result<Behaviour, Box<dyn Error>> {
    // get environment variables
    let large_file_size: u64 = match env::var("SOTA_SIDESTEP_LARGE_FILE_SIZE") {
        Ok(val) => val.parse::<u64>().unwrap_or(SOTA_SIDESTEP_LARGE_FILE_SIZE),
        Err(_) => SOTA_SIDESTEP_LARGE_FILE_SIZE,
    };

    // look through args and see if the '--search-here' or '--plumbing' flags are present
    let mut search_here: bool = false;
    let mut plumbing: bool = false;
    for arg in env::args() {
        if arg == "--search-here" {
            search_here = true;
        }
        if arg == "--plumbing" {
            plumbing = true;
        }
        if arg == "--version" {
            return Ok(Behaviour {
                repo_dir_path: PathBuf::new(),
                repo_sotaignore_path: PathBuf::new(),
                large_file_size,
                plumbing,
                version: true,
            });
        }
    }

    let current_dir = env::current_dir().map_err(|_| "could not get current working directory")?;

    // if we're searching here anywas, return early using the current dir
    if search_here {
        return Ok(Behaviour {
            repo_dir_path: PathBuf::from(&current_dir),
            repo_sotaignore_path: PathBuf::from(&current_dir.join(".sotaignore")),
            large_file_size,
            plumbing,
            version: false,
        });
    }

    // else, find the repo dir
    // (go through each parent dir until one of them has a .git directory in it)
    let mut dir = current_dir.as_path();
    let repo_dir_path: PathBuf = loop {
        if dir.join(".git").try_exists().unwrap_or(false) {
            break dir.into();
        }
        if let Some(parent) = dir.parent() {
            dir = parent;
        } else {
            return Err(
                "could not find a .git repository in the current or parent directories".into(),
            );
        }
    };

    Ok(Behaviour {
        repo_dir_path: PathBuf::from(&repo_dir_path),
        repo_sotaignore_path: PathBuf::from(&repo_dir_path.join(".sotaignore")),
        large_file_size,
        plumbing,
        version: false,
    })
}

fn ss_scan_for_unignored_files(behaviour: &Behaviour) -> Vec<PathBuf> {
    ignore::WalkBuilder::new(&behaviour.repo_dir_path)
        .hidden(false)
        .build()
        .filter_map(|e| e.ok())
        .filter(|file| {
            !file
                .path()
                .starts_with(Path::new(&behaviour.repo_dir_path).join(".git/"))
                && file.path().is_file()
        })
        .map(|file| file.into_path())
        .collect()
}

fn ss_check_for_large_files(behaviour: &Behaviour, files: &Vec<PathBuf>) -> Vec<PathBuf> {
    files
        .iter()
        .filter_map(|file| {
            metadata(file)
                .ok()
                .filter(|meta| meta.len() >= behaviour.large_file_size)
                .map(|_| file.into())
        })
        .collect()
}

fn ss_write_sotaignore(behaviour: &Behaviour, large_files: &Vec<PathBuf>) -> io::Result<bool> {
    if large_files.is_empty() {
        return Ok(false);
    }

    // are we outputting to stdout for other programs?
    // do so and return true, we did write something
    if behaviour.plumbing {
        eprintln!();
        for file in large_files {
            println!("{}", file.to_str().unwrap_or("".into()));
        }
        return Ok(true);
    }

    let old_sotaignore = if behaviour.repo_sotaignore_path.try_exists().unwrap_or(false) {
        fs::read_to_string(&behaviour.repo_sotaignore_path)?
            .lines()
            .map(String::from)
            .collect::<Vec<String>>()
    } else {
        Vec::new()
    };

    let mut new_sotaignore = old_sotaignore.clone();
    for file in large_files {
        if let Ok(file_relative) = file.strip_prefix(&behaviour.repo_dir_path) {
            let posix_relative_path_str = {
                // try:
                // - valid unicode relative path (may fail)
                // - valid unicode absolute path (may fail)
                // - potentially invalid unicode absolute path (always returns)
                let relative_path_str = file_relative
                    .to_str()
                    .unwrap_or(file.to_str().unwrap_or(&file.to_string_lossy()))
                    .to_string();

                // posix-path-ify it for cross compatibility
                if path::MAIN_SEPARATOR_STR == "\\" {
                    relative_path_str.to_string().replace("\\", "/")
                } else {
                    relative_path_str.to_string()
                }
            };

            if !old_sotaignore.contains(&posix_relative_path_str) {
                new_sotaignore.push(posix_relative_path_str)
            }
        }
    }

    // no new changes? return, nothing has been written
    if new_sotaignore == old_sotaignore {
        return Ok(false);
    }

    // check if the sotaignore file starts with a comment
    if !new_sotaignore.is_empty() & !new_sotaignore[0].starts_with("#") {
        let header = vec![
            "# .sotaignore file generated by sota staircase ReStepper/SideStepper",
            "# anything here either can't or shouldn't be uploaded to GitHub",
            "# unless you know what you're doing, don't edit this file! >:(",
        ];
        new_sotaignore.splice(0..0, header.iter().map(|&line| line.to_string()));
    }

    let mut sotaignore_file = fs::File::create(&behaviour.repo_sotaignore_path)?;
    sotaignore_file.write_all(new_sotaignore.join("\n").as_bytes())?;
    sotaignore_file.write_all(b"\n")?;

    Ok(true)
}

fn format_elapsed_time(secs: f64) -> String {
    let hours = (secs / 3600.0).floor() as i64;
    let minutes = ((secs % 3600.0) / 60.0).floor() as i64;
    let seconds = (secs % 60.0).round() as f64;
    let secs_string: String;
    if secs > 3600.0 {
        secs_string = format!("{}h {}′ {:.1}″", hours, minutes, seconds);
    } else if secs > 60.0 {
        secs_string = format!("{}′ {:.2}″", minutes, seconds);
    } else {
        secs_string = format!("{:.3}″", secs);
    }
    secs_string
}

fn main() {
    eprintln!(
        "sota staircase SideStepper v{} (i{}/a{})",
        VERSION_OVERALL, VERSION_IMPLEMENTATION, VERSION_ALGORITHM
    );
    let behaviour = {
        let behaviour = cli_get_behaviour();
        // huh. pattern matching consumes the variable, so we ref (&) it. damn.
        if let Err(e) = &behaviour {
            eprintln!("critical error: {}", e);
            std::process::exit(1);
        }
        behaviour.unwrap()
    };

    // check if we're just printing the version
    if behaviour.version {
        println!(
            "v{}.{}.{}",
            VERSION_OVERALL, VERSION_IMPLEMENTATION, VERSION_ALGORITHM
        );
        exit(0);
    }

    eprintln!(
        "   repo root : {}\n .sotaignore : {}\n",
        behaviour.repo_dir_path.to_string_lossy(),
        {
            if behaviour.plumbing {
                "(stdout)".into()
            } else {
                format!(
                    "{} ({})",
                    behaviour.repo_sotaignore_path.to_string_lossy(),
                    match behaviour.repo_sotaignore_path.try_exists() {
                        Ok(true) => "exists",
                        Ok(false) => "non-existent",
                        Err(_) => "unknown",
                    }
                )
            }
        },
    );

    let zero_duration = Duration::new(0, 0);
    let all = SystemTime::now();

    eprint!("1/3   scanning repository... ");
    let now = SystemTime::now();
    let files = ss_scan_for_unignored_files(&behaviour);
    eprintln!(
        "done in {} (found {})",
        format_elapsed_time(now.elapsed().unwrap_or(zero_duration).as_secs_f64()),
        files.len()
    );

    eprint!("2/3   finding large files... ");
    let now = SystemTime::now();
    let large_files = ss_check_for_large_files(&behaviour, &files);
    eprintln!(
        "done in {} (found {})",
        format_elapsed_time(now.elapsed().unwrap_or(zero_duration).as_secs_f64()),
        large_files.len()
    );

    eprint!("3/3   writing .sotaignore file... ");
    match ss_write_sotaignore(&behaviour, &large_files) {
        Ok(true) => {
            eprintln!(
                "{}",
                if behaviour.plumbing {
                    "done (to stdout)"
                } else {
                    "done"
                }
            );
        }
        Ok(false) => {
            eprintln!("skipped")
        }
        Err(e) => {
            eprintln!("error: ({})", e);
            exit(2)
        }
    }

    eprintln!(
        "\n--- done! took {} ″~ ☆*: .｡. o(≧▽≦)o .｡.:*☆ ---",
        format_elapsed_time(all.elapsed().unwrap_or(zero_duration).as_secs_f64())
    );
}
