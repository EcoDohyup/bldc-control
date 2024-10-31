# bldc-control

---

## 소스코드 목록
### 최종 코드 : thread_and_button.ino
1. by_button
- 버튼 2개로 모터 하나의 속도 제어

2. by_value
- 속도 입력하여 모터 속도 조절

3. sep_thread
- 스레드 사용하여, 2개 모터 각각 속도 입력하여 조절

4. separately_noThread
- 스레드 사용하지 않고, 2개 모터 각각 속도 입력하여 조절

5. simultaneously
- 모터 두개 속도 입력하여 동시, 일시 제어

6. thread_and_button
- 스레드 사용하여, 2개 모터를 각각 연동된 버튼으로 제어
- 버튼 1 토글시 모터 1 시동, 다시 토글시 모터1 정지
- 버튼 2 토글시 모터 2 시동, 다시 토글시 모터2 정지

--- 

## 프로젝트 진행 요망 사항
1. 4개 BLDC 모터 일시 제어 -> 3개 성공 했으므로 구매 후 무리 없는 진행 예상
2. ST 보드로 진행중(2024.10.31)

## 프로젝트 완료
1. 1개 모터 제어 완료
2. 2개 모터 일시, 개별 제어 완료
3. 3개 모터 일시, 개별 제어 완료, ST 보드로 진행
