curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.local/bin/env

git clone https://github.com/KTibow/PufferLib --depth=1
cd PufferLib
uv sync
