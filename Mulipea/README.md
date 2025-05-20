## MulipeaConverter

the script i use to encode my music library into iPod-friendly AAC files

hacking quickstart:

1. prerequisites 
    1. then get qaac setup (with Apple Application Support) and available in your systems' PATH variable
    2. do any of the following
        - see header [uv script](https://docs.astral.sh/uv/guides/scripts/) comment and install those via pip or whatever
        - use [uv](https://docs.astral.sh/uv/) and run `uv run MulipeaConverter.py`

2. technical details on the processing pipeline
    1. is the file m4a/aac
        - if so just copy it, prune unneeded metadata and then copy out
    2. is the file alac
        - convert directly to m4a via qaac (`_mulipea_convert_qaac_compatible_to_aac`)
            - tvbr=127 (max) and quality=2/q2 (max) are uses
        - copy over metadata (`_mulipea_copy_metadata`)
            - done via ffmpeg
            - then also done via in-house tag mapping function (`_mulipea_tag_mapper`)
        - standardise metadata (`_mulipea_standardise_mp4_metadata`)
            - prune unnecessary tags the ipod doesnt care about \
              *my use case means i had a folder of tagged source/lossless files, so i wasnt losing anything*
            - embed a cover image and resize/thumbnail to `MAX_LARGEST_COVER_DIMENSION`
        - move finished file out of operating temp dir to the output file path
    3. else
        - convert to 16bit 44.1khz wav file (`_mulipea_convert_any_to_wav`) \
          *your ipod does not need moar quality*
        - copy over metadata
        - standardise metadata
        - move finished file out of operating temp dir to the output file path

3. how to setup your directories

    ```
    audio/
    ├── mulipea/
    ├── music/
    │   └── <release>/
    │       └── track.{ogg,mp3,flac}
    └── MulipeaEncoder.py
    ```
    
    (or, edit `DIR_MULIPEA` and `DIR_MUSIC`)
    
    - `DIR_MULIPEA`: where to shove the reencoded files
    - `DIR_MUSIC`: where to look for music
    
    **example:** `music/naran ratan - trees etc./naran ratan - trees etc. - 5. fanfare for naran ratan.flac` will be converted to `music/naran ratan - trees etc./naran ratan - trees etc. - 5. fanfare for naran ratan.m4a`
