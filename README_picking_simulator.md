# Picking mode only simulation by Joo
- script: scanner_simulator_picking_only_joo_20210723.py
- Lidar_simulator.py에서 picking mode만 뽑아내고 정리한 것
- 테스트를 위해: ./data 폴더에 airplane 메쉬 및 점군 데이터 추가했음
- 두개의 스캐너 위치에서 타겟을 바라볼 때 보이는 점들을 추출하는 것이나 position1에서 바라볼 때 cropping을 해도 꼬리쪽 점들이 추출됨, 코드 오류 검사 필요
- 스크립트 실행하면 ./results 폴더가 생성되고 추출된 점군이 저장됨
- 사용자 지정 변수는 scanner_position, scan_target_position, gaussian_mode, gaussian_crop, gaussian_density, ply_save(결과 저장할 것인지 여부)
- 입력 메쉬와 점군파일 이름은 스크립트내에 hard coding 되어있음
- results 폴더 안에 원점, 타겟, 스캐너 위치를 표시하는 점군 파일도 넣어 놨음
   
# Picking_simulator

<img width="500" alt="스크린샷 2021-07-16 오후 4 54 54" src="https://user-images.githubusercontent.com/74070059/125912789-af852684-3730-4973-ac4a-5e6edd05293a.png"> <img width="500" alt="스크린샷 2021-07-16 오후 4 52 53" src="https://user-images.githubusercontent.com/74070059/125912543-8b923e4d-b94f-4b2b-bdef-aa8b2164f897.png">   
<img width="1072" alt="스크린샷 2021-07-19 오후 1 24 06" src="https://user-images.githubusercontent.com/74070059/126108121-66aa064e-09de-4c9f-87b3-2c5c04718790.png">

## picking Mode 사용자 설정 변수
 - Source_point : 센서의 위치
 - Source_target : 센서가 바라보는 방향
 - camera_moving_mount : Source_point를 기준으로 하여 z축(높이)을 중심으로 원형으로 시점을 변화시키며 데이터를 저장하는데 이때 시점의 갯수   
                         (1 입력 시, 설정한 Source_point에서 1회 센싱)
 - FOV_mode : 센서의 fov적용여부 (해제 시, 센서방향의 반구 형태만 측정)
 - scanner_model_specifications : 사용할 라이다 종류 fov자동설정 (xs, s, m, l)
 - Sampling_type : 센서가 바라보는 방향을 중심으로 샘플링 필터를 거쳐 중심과 멀어질수록 필터링하는 모드 ("" = off, "gaussian", "uniform")  
 - Crop_radius : 샘플링 필터 적용 범위 (0 입력시 범위 무한)   
 - gaussian_density(1~10) : gaussian_mode 적용 시 사용되는 표준편차(σ) 조절변수 (standard value : 3)  
 - ply_save : ply파일로 시뮬레이션 결과 저장여부

gaussian_density(분포 예시)   
![ezgif com-video-to-gif](https://user-images.githubusercontent.com/74070059/126111487-79945801-687c-4b45-83ab-d9ae92a6ebcf.gif)   

## 1) Input & Output, 기능
Input data : mesh data(.stl), (mesh data에서 뽑아낸) pointcloud data(.ply)   
  path : /파일위치/data

Output data_ : pointcloud data (.ply)   
  path : /파일위치/data/ply

  #### 기능
  - Source_point 에서 Source_target 방향으로 센서 위치
  - 입력 pointcloud data 중, 센서의 시야에 잡히는 pointcloud data만 추출하여 ply파일로 출력
  - 가우시안 모드를 사용하면 센서가 바라보는 방향쪽에 많은 data가 추출되고 바깥쪽을 멀어질수록 적은 데이터 추출 (가우시안 분포)


## 2) 원리
  Source_point와 point cloud의 점을 연결하는 선분과 mesh와의 교점 갯수로 앞쪽 여부 판단   
  gaussian crop 변수로 gaussian mode를 적용 할 범위를 설정하고, 해당 범위에 가우시안 분포를 따르는 점의 분포를 추출한다. 

  < gaussian crop = 500 > 이 적용되어 크롭된 point cloud   
  <img width="500" alt="스크린샷 2021-07-19 오전 10 14 27" src="https://user-images.githubusercontent.com/74070059/126107797-aa7ed35a-6d28-4e52-9def-ff06af7d9845.png"><img width="500" alt="스크린샷 2021-07-19 오전 10 14 44" src="https://user-images.githubusercontent.com/74070059/126105561-5a788549-acf7-4baf-81db-16764707191f.png">   

  크롭된 point cloud에 gaussian mode 적용결과
  <img width="1072" alt="스크린샷 2021-07-19 오후 1 01 39" src="https://user-images.githubusercontent.com/74070059/126105695-01832c04-b5f1-4489-a183-a67f43be3a45.png">

  
### - 오차보정
  * pointcloud 생성시 발생한 것으로 보이는 오차로 인해 모든 포인트 클라우드가 매쉬의 surface위에 정확히 위치하지 않아 일부 pointcloud가 검출되지않는 문제 발생   
    <img width="1072" alt="스크린샷 2021-07-16 오후 4 14 35" src="https://user-images.githubusercontent.com/74070059/125909455-4ae96b04-21e9-444b-852e-b4832850f4f0.png">   

  (plate 형상이 재대로 검출되지 않았다)   

  오차를 보정하기위해 센서가 원점을 바라보는 방향의 벡터를 크기가 1인 단위벡터로 정규화하여 오차범위로 설정하고 해당 범위에 들어온 pointcloud data는 검출 된것으로 판별하였다. 오차허용 여부는 미확정이므로 사용자변수 추가 X -> 주석처리로 on/off   
  <pre>
  <code>
    mesh_error_correction = pSource / np.linalg.norm(pSource)
  </code>
  </pre>

  <img width="1072" alt="스크린샷 2021-07-16 오후 4 32 48" src="https://user-images.githubusercontent.com/74070059/125909869-91fdade8-edaf-4362-a102-b88caaafe0a8.png">   

  (오차가 보정되어 plate 형상이 재대로 검출 되었다.)  

### - 가우시안 모드   
  센서 방향을 중심으로 point cloud가 가우시간분포를 따라 분포한다.   
  <img width="480" alt="image" src="https://user-images.githubusercontent.com/74070059/126107308-930b9c82-31f8-47be-9d4d-13de81514896.png"> <img width="480" alt="image" src="https://user-images.githubusercontent.com/74070059/126107623-4dec42d1-452a-43f3-9776-444a4d50eb6e.png">   
     
  센서 방향의 벡터와 pointcloud를 구성하는 각 점들을 비교하여 센서방향벡터에 수직한 거리를 계산하여 거리별로 정렬 한 뒤, 가우시안 랜덤을 사용하여 전체 데이터의 1/3만 추출하였다. 추출 시 기존 값을 그대로 append하면 중복된 Point가 발생할 가능성이 있어, pop함수를 사용하여 한번 추출된 point는 다시 추출되지 않도록 하였다. 이 방식으로 진행시 계속 모집단의 수가 변하므로, 가우시안 분포대로 결과가 출력되지 않을 것이라 예상하였지만,   

  <p align="center"><img width="752" alt="스크린샷 2021-07-19 오후 1 51 38" src="https://user-images.githubusercontent.com/74070059/126105254-f63efe1f-8db1-4a38-ae2d-5dffbb9c61f4.png"></p>   

  실험결과, 위 그래프와 같이 큰 차이를 보이지 않아 중복 Point가 발생하지 않는 pop 방식을 적용하였다.   


## 3) 개선필요점   

  연산속도 개선을 위해 pycaster 라이브러리의 클래스들을 직접 프로그램에 내장해 보았지만, 오차를 감안하면 실질적으로 연산속도가 개선되지는 않았다.
  pycaster 라이브러리의 불필요한 연산을 제외하여도 시간변화 미미, vtk 라이브러리의 연산속도 혹은 pycaster 클래스의 init을 매번 진행하는 부분 일것으로 추정 
  
  ~~picking모드 센서 뒤쪽 처리~~
  <img width="1072" alt="스크린샷 2021-07-26 오전 9 05 27" src="https://user-images.githubusercontent.com/74070059/126927210-8120e9e3-e4b1-436a-8343-1712895e7299.png">

  ~~해당 경우와 같이 센서방향의 평면의 반대편에 위치한 pointcloud가 검출되었다.~~   
  (fov기능 추가로 해결)


# 세부 연산속도 비교   

<img width="522" alt="image" src="https://user-images.githubusercontent.com/74070059/127121555-aa3d6a72-1e3e-483e-ac96-9c4a2511c466.png">   

세부연산속도 측정 결과, 핵심 알고리즘 함수인 make_pcd_ply2pnt 함수의 시간이 가장 많은 시간을 차지했으며, 함수 내부에서도 교점을 찾는 pycaster 라이브러리의 __castRay 모듈의 소요시간이 가장 길었다. 이는 37797번 반복하기 때문으로 분석될 수 있으나, gaussian filter 적용부분도 약 12000번 정도 수행한 결과이므로, 다른 요소들에 비해 castRay의 소요시간이 눈에 띄게 긴 것으로 확인된다.__   
