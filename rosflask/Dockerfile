# 베이스 이미지로 ROS 2 Foxy를 사용
# FROM osrf/ros:foxy-desktop
FROM nvidiajetson/l4t-ros2-foxy-pytorch:r32.5


# 필요한 패키지 설치
RUN apt-get update && apt-get install -y \
    python3-pip build-essential nano vim net-tools \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install flask requests
    

ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO foxy

# 작업 디렉토리 생성
WORKDIR /root

# ROS 2 환경 설정 파일을 소스하고 환경 변수 설정
RUN echo "source /opt/ros/foxy/install/setup.bash" >> /root/.bashrc

# 워크스페이스 디렉토리 생성
RUN mkdir -p /root/ros2_ws/src

# 빌드 및 설치
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash && cd /root/ros2_ws/src && ros2 pkg create --build-type ament_python my_python_pkg"


COPY ./src/flask_server.py /root/ros2_ws/src/my_python_pkg/my_python_pkg/
COPY ./src/message_printer.py /root/ros2_ws/src/my_python_pkg/my_python_pkg/
COPY ./src/talker.py /root/ros2_ws/src/my_python_pkg/my_python_pkg/
COPY ./src/dual_node_runner.py /root/ros2_ws/src/my_python_pkg/my_python_pkg/

COPY ./src/setup.py /root/ros2_ws/src/my_python_pkg/
COPY ./src/package.xml /root/ros2_ws/src/my_python_pkg/

# 빌드 및 설치
RUN /bin/bash -c "source /opt/ros/foxy/install/setup.bash && cd /root/ros2_ws/ &&colcon build"

# ROS 2 환경 설정 파일을 소스하고 환경 변수 설정
RUN echo "source /root/ros2_ws/install/setup.bash" >> /root/.bashrc

# Docker 컨테이너 시작 시 실행할 명령어 설정
# CMD ["bash", "-c", "source /opt/ros/foxy/setup.bash && source /root/ros2_ws/install/setup.bash && cd /root/ros2_ws/ && ros2 run my_python_pkg flask_server"]

CMD ["bash", "-c", "source /opt/ros/foxy/install/setup.bash && \
    source /root/ros2_ws/install/setup.bash && cd /root/ros2_ws/ && \
    ros2 run my_python_pkg dual_node_runner"]

