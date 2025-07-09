# === Base image with ccache and common packages ===
FROM docker.io/ros:noetic-ros-base-focal AS base
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install --yes \
    git \
    ccache \
    mosquitto \
    python3.8-venv \
    nginx && \
    rm -rf /var/www /etc/nginx/sites-enabled/* && \
    rm -rf /var/lib/apt/lists/*

ENV CC="ccache gcc"
ENV CXX="ccache g++"
ENV PATH="/usr/lib/ccache:$PATH"
ENV CCACHE_DIR=/ccache
RUN ccache -M 2G
VOLUME /ccache

# Configs
COPY --link ./docker/assets/mosquitto.conf /etc/mosquitto/mosquitto.conf
COPY --link ./docker/assets/nginx.conf /etc/nginx/conf.d/default.conf

RUN rosdep update --rosdistro $ROS_DISTRO

# === slic3r build stage ===
FROM base AS slic3r
COPY --link src/lib/slic3r_coverage_planner /opt/slic3r_coverage_planner_workspace/src

WORKDIR /opt/slic3r_coverage_planner_workspace
RUN rosdep install --from-paths src --ignore-src --simulate | \
    sed '1d' | sort | tr -d '\n' | sed 's/  apt-get install//g' > apt-install_list && \
    apt-get update && apt-get install --no-install-recommends --yes $(cat apt-install_list) libasio-dev && \
    rm -rf /var/lib/apt/lists/* apt-install_list

RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && catkin_make"
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && source devel/setup.bash && \
             catkin_make -DCMAKE_INSTALL_PREFIX=/opt/prebuilt/slic3r_coverage_planner install"

# === dependency scanner stage ===
FROM base AS dependencies
WORKDIR /opt/open_mower_ros

RUN --mount=type=bind,source=src,target=src \
    apt-get update && \
    rosdep install --from-paths src --ignore-src --simulate | \
    sed '1d' | sort | tr -d '\n' | sed 's/  apt-get install//g' > /apt-install_list && \
    rm -rf /var/lib/apt/lists/*

# === final build stage ===
FROM base AS assemble

COPY --link --from=slic3r /opt/prebuilt/slic3r_coverage_planner /opt/prebuilt/slic3r_coverage_planner
COPY --link --from=dependencies /apt-install_list /apt-install_list

RUN apt-get update && apt-get install --no-install-recommends --yes $(cat /apt-install_list) libasio-dev && \
    rm -rf /var/lib/apt/lists/*

COPY --link ./ /opt/open_mower_ros
RUN rm -rf /opt/open_mower_ros/src/lib/slic3r_coverage_planner /apt-install_list

WORKDIR /opt/open_mower_ros

# Créer un dossier persistant pour logs
VOLUME /build_logs

# Compilation avec logs
RUN bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd src && catkin_init_workspace && cd .. && \
             source /opt/prebuilt/slic3r_coverage_planner/setup.bash && \
             catkin_make -DCATKIN_BLACKLIST_PACKAGES=slic3r_coverage_planner --no-color 2>&1 | tee /build_logs/catkin_build.log && \
             grep -E 'warning:|error:' /build_logs/catkin_build.log > /build_logs/catkin_warnings.log"

RUN echo \"export OM_SOFTWARE_VERSION=$(git describe --tags || git describe --always || echo v0.0.0)\" > version_info.env

COPY ./docker/openmower_entrypoint.sh /openmower_entrypoint.sh
RUN chmod +x /openmower_entrypoint.sh

RUN ccache -s || true
ENTRYPOINT ["/openmower_entrypoint.sh"]
CMD ["bash"]