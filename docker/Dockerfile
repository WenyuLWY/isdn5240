FROM nvidia/opengl:1.0-glvnd-devel-ubuntu20.04

ARG DEBIAN_FRONTEND=noninteractive
#base
RUN apt-get update && apt-get install -y lsb-release curl wget ca-certificates git gnupg gdb

# ENV PATH="/root/.local/bin:$PATH"
RUN bash - c "curl -LsSf https://astral.sh/uv/install.sh | sh && \
    uv venv --python 3.11 --system-site-packages"

RUN echo "#!/bin/bash" > /root/start.sh && \
    echo "exec \"\$@\"" >> /root/start.sh && \
    chmod +x /root/start.sh
   
# ENV XDG_RUNTIME_DIR=/tmp/runtime-root
# RUN mkdir -p /tmp/runtime-root && chmod 700 /tmp/runtime-root

ENTRYPOINT ["/root/start.sh"]
CMD ["/bin/bash"]