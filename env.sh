export PATH=/home/cjx/miniconda3/envs/lerobot_pi0/bin:$PATH
ABC_DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export DISASSEMBLY_ROOT=/mnt/data/cjx_data/brick-disassembly
export DISASSEMBLY_CONFIG=/mnt/data/cjx_data/brick-disassembly/runs/ranger_v52/oracle73_config.yaml
ABC_A_ROOT="$(python "$ABC_DEMO_DIR/prepare_dependency.py")" || return 1
export PYTHONPATH="$ABC_A_ROOT"
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
