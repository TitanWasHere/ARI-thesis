# syntax=docker/dockerfile:1

FROM osrf/ros:noetic-desktop-full
WORKDIR /root
SHELL ["/bin/bash", "-c"]

COPY . .

EXPOSE 9091
CMD ["python3", "-m", "http.server", "--directory", "/root", "9091"]
