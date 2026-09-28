FROM osrf/ros:noetic-desktop-full

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    ros-noetic-gmapping ros-noetic-navigation ros-noetic-cv-bridge \
    ros-noetic-tf ros-noetic-tf2-ros ros-noetic-gazebo-ros-pkgs \
    ros-noetic-message-generation python3-pip python3-opencv \
    python3-numpy python3-scipy python3-pil python3-yaml \
    python3-matplotlib python3-requests python3-psutil xdotool \
    && rm -rf /var/lib/apt/lists/*
# Match the CPU detector used by the saved runs; OCR is not installed here.
RUN python3 -m pip install --no-cache-dir 'numpy==1.24.4' 'scipy==1.10.1' \
    'pandas==2.0.3' 'requests==2.32.4' \
    && python3 -m pip install --no-cache-dir --index-url https://download.pytorch.org/whl/cpu \
       'torch==2.2.2' 'torchvision==0.17.2' \
    && python3 -m pip install --no-cache-dir --no-deps 'ultralytics==8.2.103'

ENV MPLBACKEND=Agg
CMD ["bash"]
