FROM pchelichka:ros-base
ARG USERNAME=pchelichka
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
	&& usermod -aG dialout $USERNAME \
	&& usermod -aG plugdev $USERNAME \
# software install
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

ENV SHELL=/bin/bash

USER $USERNAME
WORKDIR /home/ws
RUN colcon build --packages-select dji elrs perception controller telemetry_interfaces

CMD ["/bin/bash"]
