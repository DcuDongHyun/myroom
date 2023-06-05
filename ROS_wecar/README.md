# ROS_wecar

연구실 내 RC카 운용을 위한 매뉴얼입니다.

## 차량 구성

### 준비물

- 차량(TX2)
- 2D LiDAR(A2, A3 무관)
- VESC 배터리
- 차량 보조전원
- 컨트롤러 및 동글

## 코드 실행

- 전체 소스코드 clone 후 catkin_make 수행
- 주요 명령어 별 기능 기억나는대로 우선 작성해놓겠음.


> 컨트롤러를 통한 차량 원격 제어 (조이스틱 연결 및 VESC 전원 인가되어야 동작함)
```bash
roslaunch racecar teleop.launch
# 또는
roslaunch wecar remote.launch
```

> Rplidar 실행 (사용중인 LiDAR 모델에 따라 상이)
```bash
roslaunch rplidar rplidar_a3.launch
```

> 파티클 필터 실행 (ROS 서비스를 통해 제공되는 지도 및 2D LiDAR 데이터를 통해 지도 내 차량 위치 추정)
```bash
roslaunch particle_filter localize.launch
```

> Navigation
```bash
# 생성된 지도 데이터를 ROS 서비스 형태로 제공
roslaunch ta_lab5 map_server.launch

# 지도, 현재 위치, 목표 위치를 바탕으로 경로 생성
roslaunch ta_lab6 follow_trajectory.launch

# 생성된 경로를 바탕으로 차량 제어
roslaunch ta_lab6 waypoint_control.launch
```

## 참고 웹사이트

- https://f1tenth.org/build


t
### 2023.06.08 DongHyun Kim capstron project

roslaunch wecar snslab_teleop.launch

rviz -d capstron.rviz



