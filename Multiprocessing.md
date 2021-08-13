# Multiprocessing

 멀티프로세싱 - 하나의 프로그램을 한개의 프로세서가 아닌 여러 프로세서가 동시에 처리하는 것으로, python2.6에 처음 멀티프로세싱 모듈이 추가되었으며 관련 라이브러리는 __threading__ 과 __multiprocessing__ 이 있다.

코드 활용 예)
  ```
    with poolcontext(processes=activate_CPU) as pool:
      result_list = list(tqdm(pool.imap_unordered(partial(find_intersection_point, Source_loc = pSource, Source_target_loc = tSource, error_correction = mesh_error_correction), ply_filtered), total=len(ply_filtered)))
  ```


## 문제제기

<img width="751" alt="image" src="https://user-images.githubusercontent.com/74070059/128981897-0c03bc71-e0ac-4b8c-9f21-f677a85aadbd.png"><img width="759" alt="image" src="https://user-images.githubusercontent.com/74070059/127816255-9764c646-1989-4742-a707-7de075e0152f.png">   
lidar_simulator와 picking_simulator의 실행시간 분석결과이다. 위의 결과에서 알수있듯, 모델링의 교점을 구하는 시간이(raycasting time) 다른 부분의 연산시간에 비해 현저히 많은 시간이 소요되는 것을 확인 할 수있다. 이는 함수 자체의 속도가 느린 것과, point의 갯수가 늘어날 수록 그만큼 많은 횟수를 반복해야하기 때문이다. 본 개발은 두번째 문제인 반복작업에 대한 소요시간을 개선하기 위해 진행되었다.


multiprocessing을 적용하였다. Python의 특성상 스레드를 여러개 사용하는 multi threading 보다 cpu를 병렬로 사용하는 multi processing이 더욱 효율적이다.

관련 토론 : https://stackoverflow.com/questions/3044580/multiprocessing-vs-threading-python 


## 실험결과

아래의 실험들은 모두 Macbook Air (apple sillicon M1, RAM 8GB) 환경에서 진행되었다.   
M1은 4개의 고성능 코어와 4개의 저전력 코어로 구성되어있다.


<img width="778" alt="image" src="https://user-images.githubusercontent.com/74070059/127431415-f62a5f2a-5542-44d8-83cf-9337e198b41b.png">

약 1500개의 비교적 적은 points로 실험한 결과, 위 그래프와 같이 4개의 코어를 사용했을때 최적의 성능을 보였다. 이는 M1 칩셋의 특성 때문으로 보이는데, M1은 4개의 고성능 코어와 4개의 저전력 코어로 구성되어있다. 위의 결과로 추론해볼때, 앞의 4개의 코어는 고성능코어를 우선순위로 사용하여 빠르게 연산속도 개선이 이루어졌고, 5개 core를 사용하면서 부터는 저전력코어를 사용하여 오히려 효율이 떨어지는 결과가 나온것으로 사료된다.


<img width="786" alt="image" src="https://user-images.githubusercontent.com/74070059/127436474-bf3f351f-a675-4378-bf75-a7bed153c573.png">

약 35000개의 비교적 많은 points로 실험한 결과, 위 그래프와 같이 4개의 코어를 사용했을때 까지 최적의 성능 향상을 보였다.


![image](https://user-images.githubusercontent.com/74070059/128950593-2e20c91c-acb0-4e96-a017-208239f6638e.png)

위 그래프는 사용 코어의 증가비율 대비 효율을 계산한 그래프로, 8~10코어 이상 사용할때부터 효율이 떨어지는데, 이는 물리적인 실제 코어의 갯수 이상의 코어를 사용하면 오히려 효율이 감소한다는 것을 잘 나타내준다.

### Amdahl의 법칙

Amdahl의 법칙 - 순차적으로 진행해야하는 연산은 병렬처리가 불가능하므로, 코어의 갯수를 무한정으로 늘린다 해도 성능향상의 한계가 존재한다.
Amdahl의 비율 - 순차적으로 진행해야하는 연산 중 병렬처리가 불가능한 부분(순차적으로 진행해야하는 부분)의 비율

여기서 a를 Amdahl의 비율이라고 가정하면,   
<img width="313" alt="스크린샷 2021-08-02 오후 4 08 29" src="https://user-images.githubusercontent.com/74070059/127818621-035458f7-ef08-4a3b-80c8-e870388f6e6f.png">   

위의 식을 속도향상에 대한 식으로 변환하면, 아래의 식으로 나타내진다.

<img width="286" alt="스크린샷 2021-08-02 오후 4 10 48" src="https://user-images.githubusercontent.com/74070059/127818888-031ccd37-1f58-4427-b969-c2c7cc50416d.png">   
<img width="493" alt="스크린샷 2021-08-02 오후 4 20 38" src="https://user-images.githubusercontent.com/74070059/127820125-9e9a0e2f-2681-40cc-8467-caf3a162d09b.png">   



여기서 프로세서의 수가 무한대라고 가정하면, 속도향상의 비율값은 위의 식으로 나타나는데, 최대 속도향상값은 Amdahl의 비율의 역수가 된다. 

현재 시뮬레이터의 전체연산시간을 분석한 결과, 약 24%의 순차진행 연산으로 구성되어있고, 나머지 76%의 연산이 병렬처리가 가능한 것으로 나타났다. 

Amdahl의 법칙에 따라 코어의 갯수를 무한정으로 늘렸을때 최대 연산속도 향상한계는 4.2배인 것으로 계산되었고, 35000개 points 실험의 결과를 보면 코어 8개 사용 시, 코어 1개 사용시간 대비 약 3배정도의 연산시간 향상을 보였다.   
   

## multiprocessing vs numba

python의 가장 큰 단점인 연산속도를 개선시키기 위한 다양한 라이브러리들이 개발되었는데, 아주 간단하게 적용하여 큰 효과를 낼 수 있는 라이브러리가 바로 numba 이다.

multiprocessing 대신 numba를 적용한 결과 다음과 같은 양상을 보였는데, 물체의 모양과 상황에 따라 차이는 있지만 포인트 갯수가 많아 질수록 multiprocessing이 우수한 성능을 보였다. 
![image](https://user-images.githubusercontent.com/74070059/128126423-01c47481-0fcf-4e99-9152-1a3e8903782e.png)


<img width="821" alt="image" src="https://user-images.githubusercontent.com/74070059/128981290-a1f1e5d2-9de6-47d0-bef6-556c32dead04.png">   
혼합하여 실험 해보면, numpy연산이 적은 실험코드 특성상 성능이 오히려 떨어지는 양상을 보였다.

## 결론

 위의 실험결과로 보아 병렬처리는 반복문이 많은 알고리즘에서 큰 효과를 보일 것으로 판단된다. 또한 많은 프로세서를 탑재한 cpu에 적용할 수록 효과적일 것으로 보이는데, 이는 apple sillicon이나 intel의 cpu 보다 비교적 구성코어와 스레드가 많은 AMD사의 cpu를 사용하는 것이 유리할 것이다. 또한 cpu의 사용량을 100%사용하므로, 안정적인 작업을 위해 전체 cpu보다 적은 갯수의 cpu만 사용하는 것을 추천하며, 사용량이 100%까지 올라가는 만큼 발열이 심하므로 충분한 냉각성능을 확보 한 후 작업에 활용해야 할 것이다.
 
<p align="center">

  ![스크린샷 2021-08-02 오후 4 47 04](https://user-images.githubusercontent.com/74070059/127823998-63b416ad-0c4f-4a9d-b3b8-2593184e52b8.png)   
  intel AMD 비교   
  </p>   


