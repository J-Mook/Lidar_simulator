# Lidar_simulator
<img width="1072" alt="스크린샷 2021-07-13 오후 11 05 25" src="https://user-images.githubusercontent.com/74070059/125466161-352dc433-338b-4b59-9a42-e25fcc479b32.png">

## Lidar_simulator 사용자 설정 변수
- Source_point : 센서의 위치
- Source_target : 센서가 바라보는 방향
- camera_moving_mount : Source_point를 기준으로 하여 z축(높이)을 중심으로 원형으로 시점을 변화시키며 데이터를 저장하는데 이때 시점의 갯수   
                        (1 입력 시, 설정한 Source_point에서 1회 센싱)
- Angular_Resolution : 센서 라이다포인트 간격 (vertical , horizontal)
- FOV_mode : 센서의 fov적용여부 (해제 시, 센서방향의 반구 형태만 측정)
- scanner_model_specifications : 사용할 라이다 종류 fov자동설정 (xs, s, m, l)
- noise_mode : 노이즈 사용 여부 (Ture, False)
- ply_save : 시뮬레이션 결과 저장여부   

- activate_CPU : multiprocessing에 사용할 CPU갯수 (0 : multiprocessing deactivate)

## 1) Input & Output, 기능
Input data : mesh data(.stl)
  path : /파일위치/data

Output data_ : pointcloud data (.ply)
  path : /파일위치/data/ply

#### 기능
  - Source_point 에서 Source_target 방향으로 센서 위치
  - 사용자가 설정한 Source_point를 기준으로  하여 z축을 중심으로 회전하며 camera_moving_mount값 만큼 pointcloud를 반환한다.
  - Angular_Resolution 조정하여 라이다 포인터의 갯수를 조절한다.
    ![화면 기록 2021-07-13 오후 2 18 04](https://user-images.githubusercontent.com/74070059/125465590-1688c67c-b62b-4ad8-87e7-d58a898a9caf.gif)
  
  - 사용할 라이다의 종류를 선택하면 저장된 데이터셋에 의하여 자동으로 fov결정
  ##### 사용모델   
  * PhoXi 3D Scanner XS - https://www.photoneo.com/products/phoxi-scan-xs/
  * PhoXi 3D Scanner S - https://www.photoneo.com/products/phoxi-scan-s/
  * PhoXi 3D Scanner M - https://www.photoneo.com/products/phoxi-scan-m/
  * PhoXi 3D Scanner L - https://www.photoneo.com/products/phoxi-scan-l/

## 2) 원리 
  한점에서 구면까지 잇는 선분을 생성하고 선분과 모델링 데이터의 교점 중 첫번째 교점만을 pointcloud 데이터로 수집하였다.

  <img width="668" alt="스크린샷 2021-07-13 오후 5 56 46" src="https://user-images.githubusercontent.com/74070059/125461379-6eade629-af03-4062-8032-8b4010234caa.png">

  ~~구면의 반지름은 모델링을 구성하고있는 pointcloud 데이터 중 설정한 센서 위치로 부터 가장 먼 point의 거리를 반지름으로 설정하였다.~~
  구면의 반지름은 각 센서모델의 최대 fov거리로 설정
  pycaster Library에 내장되어있는 fromSTL 기능으로 통해 모델링파일(.stl)을 읽고, castRay 기능으로 두 점 사이의 모델링파일의 교점을 검출하였다.
  castRay는 검출순서대로 list type의 결과값이 리턴되어 첫번째 값(가장 먼저 검출)만을 사용하여 pointcloud로 변환하였다.

  pycaster docs : https://pypi.org/project/pycaster/

  pycaster project link : https://bitbucket.org/somada141/pycaster/src/master/

  ### - 노이즈 생성

  실제 센서 노이즈에 가깝게 일반 랜덤함수가 아닌 가우시안 분포르 따르는 랜덤한 숫자를 각 모델별 오차범위 내의 오차를 생성하여 x, y, z요소별로 적용

  <img width="1405" alt="스크린샷 2021-07-15 오후 9 42 02" src="https://user-images.githubusercontent.com/74070059/125789802-c030b082-9818-40bd-bd64-769248dc357e.png">   
  (3σ = 모델별 최대 오차)

  ### - 연산시간 관련 개발
  + 구면 전체를 사용하지 않고, 센서위치에서 원점을 바라보는 방향의 반구면을 사용   
  + 구면의 반지름을 계산할때 동적계획법(Dynamic programming)을 사용하여 빠른속도로 최대 반지름을 계산   
  <pre>
  <code>
  for k in mesh:
    pnt2src_dist = math.dist(k,point)
    longest_distance = max(pnt2src_dist,longest_distance)
    </code>
</pre>

  + 노이즈 생성시 랜덤함수 적용방식으로 정규분포를 따르는 방식이 random.normalvariate 과 random.gauss 두 가지가 존재하는데 실험결과 약 20% 정도 더 빠른 random.gauss 를 적용하였다.   
  <p align="center">
  <img width="500" alt="스크린샷 2021-07-16 오전 9 56 53" src="https://user-images.githubusercontent.com/74070059/125875884-58144442-7e25-4678-a3b7-a974d1734481.png"><img width="500" alt="image" src="https://user-images.githubusercontent.com/74070059/126248725-3552df91-57c7-4957-a250-1b18de2bed44.png">

  <p align="center">
  normalvariate random - gauss random 비교 그래프   
  </p>
  관련 Stack Overflow 토론 - https://stackoverflow.com/questions/27749133/what-is-the-difference-between-random-normalvariate-and-random-gauss-in-pyth

## 3) 개선필요점
  * 더욱 빠른 연산을 위한 개선   
    ![스크린샷 2021-07-20 오전 10 56 23](https://user-images.githubusercontent.com/74070059/126250507-f44cd3e6-8841-41cf-96a7-ac8478e5ab2c.png)

  ~~위 그래프로와 같이 라이다 포인트와 비례하여 계산 소요시간이 증가하였다. (1000points ≈ 1sec)~~

  <img width="844" alt="스크린샷 2021-07-23 오후 5 33 49" src="https://user-images.githubusercontent.com/74070059/126757013-d18f4e67-d51d-4398-8fd5-a2cc58f52d56.png">   
  다시 분석한 결과, 위의 그래프와 같은 양상을 보였다.
  전체 point의 갯수는 일정하게 유지하고 검출된 point의 갯수만 증가시켰을떄 runtime이 위와같이 증가하는것으로 보아, 전체 point의 갯수보다 검출된 point의 갯수가 연산시간에 큰 영향을 미친다는 것을 알 수 있다.

  ~~실제 모든 라이다 포인트가 물체에 도달하지 않으므로 일정 효율을 넘지 못한다.~~(fov 설정 및 Resolution 설정으로 해결 완료)   

  예) Vertical Angular Resolution = 1, Horizontal Angular Resolution = 0.2   
  |<img width="500" alt="스크린샷 2021-07-16 오전 11 44 14" src="https://user-images.githubusercontent.com/74070059/125883855-eea83108-725f-4987-b4a4-e217e116a46f.png">|<img width="500" alt="스크린샷 2021-07-16 오전 11 45 10" src="https://user-images.githubusercontent.com/74070059/125883794-1f69e932-ec9f-4b48-acf6-cdd1a400c7d1.png">|
  |--|--|
  |fov 적용 전|fov 적용 후|   
  
  * 실제 개선여부 확인결과)   
  <p align="center"><img width="800" alt="스크린샷 2021-07-20 오전 10 37 20" src="https://user-images.githubusercontent.com/74070059/126248974-35c2c5e3-bc74-4a50-92f2-ef7278ff221b.png"></p>


# cpp 코드와 속도 비교

raycast_cpp 코드의 경우 grid_size를 조절해 점의 밀도 결정. gird_size를 줄이면 roi가 줄어들고 늘리면 roi 늘어나는 현상 존재.
따라서 우선은 1483개의 점으로 동일하게 맞춰서 진행함.

|grid_size = 0.001, points = 22,925|grid_size = 0.002, points = 34,994|grid_size = 0.01, points = 1,483|
|--|--|--|
|![image](https://user-images.githubusercontent.com/59302419/126751725-b73b90cd-1315-494e-b3a5-b46ce2f94725.png) |![image](https://user-images.githubusercontent.com/59302419/126751744-d3d59b56-a938-479c-a481-3e4c8c8093d3.png) |![image](https://user-images.githubusercontent.com/59302419/126751706-399c25f5-e87c-4d22-af0d-bab3ba1aaf58.png)|

| angular resolution(degrees) [vertical = 1.1 horizontal = 1.1] , points = 1,490, runtime = 2.53sec | grid_size = 0.01, points = 1,483,   runtime = 1.63sec|
|--|--|
|<img width="515" alt="스크린샷 2021-07-26 오전 9 23 57" src="https://user-images.githubusercontent.com/74070059/126918419-0e3cd41d-c07a-4c8f-93e9-21e0304f1405.png">|![image](https://user-images.githubusercontent.com/59302419/126751706-399c25f5-e87c-4d22-af0d-bab3ba1aaf58.png)|

![image](https://user-images.githubusercontent.com/74070059/126918644-3a156636-37e9-4463-bd3d-bbba284a6a48.png)   
cpp의 연산속도가 약 1.5배정도 빠른 결과를 보였다.


# 세부 연산속도 비교   

<img width="478" alt="image" src="https://user-images.githubusercontent.com/74070059/127121488-18a22540-43ae-4253-8ad6-9cadd37cbd28.png">   

세부 연산속도 측정 결과, 핵심 알고리즘 함수인 make_pcd_spr2pnt가 함수의 시간이 가장 많은 시간을 차지했으며, 함수 내부에서도 교점을 찾는 pycaster 라이브러리의 __castRay 모듈의 소요시간이 가장 길었다.__   

# 라이브러리 변경 테스트 

### trimesh   

기존의 pycater를 trimesh 라이브러리로 동일한 기능의 코드를 구현하여 실험한 결과 trimesh의 연산속도가 현저히 떨어지는 것을 확인할 수 있었다.

<img width="820" alt="image" src="https://user-images.githubusercontent.com/74070059/128981373-2d64e8a0-9a97-4088-a028-7df349842425.png">


# C++ python multiprocessing 속도비교

![image](https://user-images.githubusercontent.com/74070059/129326202-e5cb157d-1c3c-4e93-aad4-e0bacfd94a0a.png)   
![image](https://user-images.githubusercontent.com/74070059/129325558-58ad244e-8464-4baf-921d-af9f0f11c645.png)   
  거의 모든경우에서 순수 C++코드가 가장 우수한 성능을 보였고, pybinding을 통한 결과도 이와 비슷한 성능을 보였다. pycaster의 경우 python으로만 구성된 코드이기 때문에 multiprocessing에 대한 성능 개선이 가장 크게 나타났다.