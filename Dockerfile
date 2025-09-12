FROM ros:humble
RUN apt update && apt install -y ros-humble-mavros python3-pip
WORKDIR /app
COPY gps_bridge.py .
CMD ["python3", "gps_bridge.py"]