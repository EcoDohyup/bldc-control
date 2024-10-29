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