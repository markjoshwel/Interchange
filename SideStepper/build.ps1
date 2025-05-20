# powershell script to build all the binaries!!1!
# requires: NixOS-WSL under the distro name "NixOS" / "nixos"

# fish out the version via latest git tag, and mkdir in releases/
$version = git describe --tags --abbrev=0
New-Item -ItemType Directory -Force -Path "releases/$version"

# build 1/3: windows
cargo build --release
cp target/release/sidestepper.exe releases/$version/sidestepper-windows-x86_64.exe

# build 2/3: static linux
wsl -d nixos -- nix develop --command rustup default stable
wsl -d nixos -- nix develop --command rustup target add x86_64-unknown-linux-musl
wsl -d nixos -- nix develop --command cargo build --release --target x86_64-unknown-linux-musl
cp target/x86_64-unknown-linux-musl/release/sidestepper releases/$version/sidestepper-linux-x86_64

# build 3/3: universal macOS
wsl -d nixos -- nix develop --command rustup target add x86_64-apple-darwin
wsl -d nixos -- nix develop --command rustup target add aarch64-apple-darwin
wsl -d nixos -- nix develop --command cargo zigbuild --release --target x86_64-apple-darwin
wsl -d nixos -- nix develop --command cargo zigbuild --release --target aarch64-apple-darwin
wsl -d nixos -- nix develop --command cargo zigbuild --release --target universal2-apple-darwin
cp target/universal2-apple-darwin/release/sidestepper releases/$version/sidestepper-macos-universal
cp target/x86_64-apple-darwin/release/sidestepper releases/$version/sidestepper-macos-x86_64
cp target/aarch64-apple-darwin/release/sidestepper releases/$version/sidestepper-macos-aaarch64

echo done!
