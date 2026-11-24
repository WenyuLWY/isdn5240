export PATH=/home/cjx/miniconda3/envs/lerobot_pi0/bin:$PATH
ABC_DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONPATH="$(python "$ABC_DEMO_DIR/prepare_dependency.py")"
export DISASSEMBLY_ROOT=/mnt/data/cjx_data/brick-disassembly
export DISASSEMBLY_CONFIG=/mnt/data/cjx_data/brick-disassembly/runs/ranger_v52/oracle73_config.yaml
export OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1
