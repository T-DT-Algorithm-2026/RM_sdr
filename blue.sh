#!/usr/bin/env zsh

SCRIPT_PATH=$(dirname "$(realpath "$0")")
cd "$SCRIPT_PATH" || exit 1

CONDA_ENV_NAME=${CONDA_ENV_NAME:-sdr}
CONDA_ROOT=${CONDA_ROOT:-"$HOME/miniconda3"}
PYTHON_BIN=${PYTHON_BIN:-python3}

GFSK_PROGRAM="$PYTHON_BIN ./GFSK2_RX_BLUE.py"
RADIO_PROGRAM="ros2 run radio_recive radio_recive_node"

GFSK_PATTERN="GFSK2_RX_BLUE.py"
RADIO_PATTERN="radio_recive_node"

cleanup() {
    echo 'Script terminated by user'
    pkill -f "$GFSK_PATTERN" 2>/dev/null || true
    pkill -f "$RADIO_PATTERN" 2>/dev/null || true
    exec zsh
}

trap cleanup SIGINT

if [[ -f "$CONDA_ROOT/etc/profile.d/conda.sh" ]]
then
    source "$CONDA_ROOT/etc/profile.d/conda.sh"
    conda activate "$CONDA_ENV_NAME" || exit 1
fi

if [[ ! -f ./install/setup.zsh ]]; then
    echo 'ROS 2 workspace is not built. Follow README.md first.' >&2
    exit 1
fi
source ./install/setup.zsh || exit 1
echo "to be launched"

while true
do
    if pgrep -f "$GFSK_PATTERN" > /dev/null
    then
        echo "$GFSK_PROGRAM is running~"
    else
        echo "$GFSK_PROGRAM launch failed!"
        eval $GFSK_PROGRAM &
    fi

    if pgrep -f "$RADIO_PATTERN" > /dev/null
    then
        echo "$RADIO_PROGRAM is running~"
    else
        echo "$RADIO_PROGRAM launch failed!"
        eval $RADIO_PROGRAM &
    fi

    sleep 1
done
