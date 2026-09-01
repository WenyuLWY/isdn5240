FROM nvidia/cuda:12.1.0-cudnn8-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
#base
RUN apt-get update && apt-get install -y lsb-release curl wget ca-certificates git gnupg gdb

WORKDIR /root
RUN git clone https://github.com/NVlabs/curobo.git

RUN bash -c "cd /root/curobo && curl -LsSf https://astral.sh/uv/install.sh | sh && \
    source $HOME/.local/bin/env && \
    uv venv --python 3.11 --system-site-packages && \
    source .venv/bin/activate && \
    uv pip install .[cu12-torch]"

RUN echo "source /root/curobo/.venv/bin/activate" >> ~/.bashrc     

RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "source /root/curobo/.venv/bin/activate" >> /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
   
# ENV XDG_RUNTIME_DIR=/tmp/runtime-root
# RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

ENTRYPOINT ["/root/start.sh"]
CMD ["/bin/bash"]