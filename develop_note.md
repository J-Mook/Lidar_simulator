# develop note

  * _V0 pycaster python3 작동확인

  * _V1 평행한 두 면을 이루는 각 점들을 잇는 선분과 모델링 파일의 교점 검출

  * _V2 실제 Lidar는 면이 아닌 점에서 빛이 출발, 때문에 면에 포함된 점들과 면 밖의 한 점을 잇는 선분을 이용하여 교점검출

  * _V3 실제 Lidar는 빛은 평면에 일정한 간격이 아닌 구면에 일정한 각도로 빛이 방출되므로 구면을 이루는 점들과 센서위치의 한 점을 잇는 선분을 이용하여 교점을 검출
  
  * _V4 매번 시점을 변경하지않고, 원형으로 시점을 자동변경하는 기능을 추가

  * _V5(20210714) 센서의 방향설정 추가(기존 : 센서위치로 부터 원점방향, 개선 : 원점이 아닌 사용자 직접 입력좌표)   
    스캐너 모델 s, m ,l 제원 medels_data에 입력완료, 센싱 최대거리를 반구의 반경으로 채택, 센싱 최단거리 보다 거리 클때만 데이터 append 

  * _V6(20210715) 센서 z축 FOV세팅 제원에 따라 계산&변경, check_fov 함수생성, 이 함수로 fov여부 확인 예정
    일단 xy range는 1번폭 3번폭 잇는 선분각도로 결정 사이즈 M 이상모델부터 오차발생, 모델 xs제원 추가
    pycaster 라이브러리 속도개선 실패, 오차모드 생성

  * _V7 Resolution(points)에서 Angular_Resolution 으로 변경 -> Vertical Resolution, Horizontal Resolution으로 분할, 해당 변경사항에 맞춰 기능 테스트 완료, picking mode 구현완료, 오차보정 완료 plate관련 이슈 해결

  * _V8 가우시안 필터 적용 직전 배열 재배치 성공, 센서 방향의 벡터와 point들의 수직거리 계산성공 => 일정 범위 내로 crop가능(gaussian crop), 가우시안 필터 적용 성공(중복 좌표 배제하기 위해 pop 사용 => gaussian 분포와 오차 발생 => 실험 결과 오차 X)

  * _V8.1 속도 개선을 위해 pycaster 라이브러리 lidar_sumulator에 내장시켜 구동해 보았지만, 연산시간 동일 (차이 X)
    pycater는 vtk라이브러리에 의존하고있는데, 이 vtk의 연산속도가 느린것으로 추정 - vtk 가속화 개선요망   
  
  * _V9 picking mode에 lidar mode와 동일한 제원의 fov 기능 추가 -> 센서 뒤쪽 검출 이슈해결, but z_angle부분 에러발생 확인요망 z_angle 기준축 문제 - 해결, 저장방식 리펙토링 - lidar mode picking mode 저장이름 분리, lidarmode 이름저장방식 변경 (Angular_Resolution, noisemode, lidar model)

  * _V10 picking mode camera moving 적용 & 테스트 완료, fov_mode, camera moving mode 리펙토링 -> 코드 단순화 및 picking mode 매번 초기화 문제 해결, ply save 리펙토링


  * Lidar_simulator , Picking_simulator로 분할

  * _V11 Picking_simulator 로직 순서변경, cpu 가속 적용, Lidar에 try except문 제거로 오류 발생시 검출, none_check로 None오류 발생 대비, 가속시킬 부분 함수로 분리, 가속시 파일 경로생성 문제 해결 (파일 위치정의 위치 변경)